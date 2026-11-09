#define _GNU_SOURCE
#include "acq_thread.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <gpiod.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#ifndef SER_RS485_ENABLED
#ifndef TIOCSRS485
#define TIOCSRS485 0x542F
#endif
#define SER_RS485_ENABLED (1 << 0)
struct serial_rs485 {
    uint32_t flags;
    uint32_t delay_rts_before_send;
    uint32_t delay_rts_after_send;
    uint32_t padding[5];
};
#endif

static app_config_t g_cfg;

#define MAX_GROUPS       32
#define MAX_GROUP_POINTS 16
#define MAX_RESP_LEN     256
#define MAX_SLIDE_BYTES  256

typedef struct {
    uint16_t start;
    uint16_t count;
    int      idx[MAX_GROUP_POINTS];
    int      point_cnt;
} poll_group_t;

static poll_group_t g_groups[MAX_GROUPS];
static int          g_group_cnt = 0;

static int               g_fd   = -1;
static struct gpiod_chip *g_chip = NULL;
static struct gpiod_line *g_de   = NULL;
static pthread_t         g_tid;
static volatile int      g_run = 0;
static pthread_mutex_t   g_mu  = PTHREAD_MUTEX_INITIALIZER;
static acq_snapshot_t    g_snap;

/* 心跳：采集线程每轮更新，monitor 线程检查 */
static atomic_llong g_heartbeat_us = 0;

static int64_t now_mono_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}
static int64_t now_utc_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}
static uint16_t crc16_modbus(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
    return crc;
}
static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    default:     return B115200;
    }
}

static int cmp_point_addr(const void *a, const void *b)
{
    int ia = *(const int *)a, ib = *(const int *)b;
    uint16_t aa = g_cfg.points[ia].addr;
    uint16_t ab = g_cfg.points[ib].addr;
    return (aa > ab) - (aa < ab);
}

static void build_groups(void)
{
    g_group_cnt = 0;
    int order[MAX_POINTS];
    for (int i = 0; i < g_cfg.point_count; i++) order[i] = i;
    qsort(order, g_cfg.point_count, sizeof(int), cmp_point_addr);

    int i = 0;
    while (i < g_cfg.point_count && g_group_cnt < MAX_GROUPS) {
        poll_group_t *g = &g_groups[g_group_cnt++];
        memset(g, 0, sizeof(*g));
        int first = order[i];
        g->start = g_cfg.points[first].addr;
        g->count = 1;
        g->idx[g->point_cnt++] = first;

        int j = i + 1;
        while (j < g_cfg.point_count && g->point_cnt < MAX_GROUP_POINTS) {
            int cur = order[j];
            if (g_cfg.points[cur].type > PT_UINT16) break;
            if (g_cfg.points[cur].addr != g->start + g->count) break;
            g->idx[g->point_cnt++] = cur;
            g->count++;
            j++;
        }
        i = j;
    }
    fprintf(stderr, "[acq] built %d read group(s)\n", g_group_cnt);
    for (int k = 0; k < g_group_cnt; k++)
        fprintf(stderr, "  group[%d] start=0x%04X count=%u points=%d\n",
                k, g_groups[k].start, g_groups[k].count, g_groups[k].point_cnt);
}

static int rs485_init(void)
{
    if (strcmp(g_cfg.mode, "gpio") != 0) {
        fprintf(stderr, "[rs485] mode=%s, skip gpio init\n", g_cfg.mode);
        return 0;
    }
    if (g_cfg.de_gpio < 0) { return -1; }
    g_chip = gpiod_chip_open("/dev/gpiochip0");
    if (!g_chip) return -1;
    g_de = gpiod_chip_get_line(g_chip, g_cfg.de_gpio);
    if (!g_de) return -1;
    int init_val = g_cfg.de_active_high ? 0 : 1;
    if (gpiod_line_request_output(g_de, "rs485-de", init_val) < 0) return -1;
    return 0;
}
static inline void rs485_tx(void)
{
    if (g_de) gpiod_line_set_value(g_de, g_cfg.de_active_high ? 1 : 0);
}
static inline void rs485_rx(void)
{
    if (g_de) gpiod_line_set_value(g_de, g_cfg.de_active_high ? 0 : 1);
}

static int serial_open(void)
{
    int fd = open(g_cfg.device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    struct termios tio;
    if (tcgetattr(fd, &tio) < 0) { close(fd); return -1; }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~PARENB;

    if (strcmp(g_cfg.parity, "even") == 0)      tio.c_cflag |= PARENB;
    else if (strcmp(g_cfg.parity, "odd") == 0)  tio.c_cflag |= (PARENB | PARODD);

    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= (g_cfg.databits == 7) ? CS7 : CS8;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    speed_t sp = baud_to_speed(g_cfg.baudrate);
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);
    if (tcsetattr(fd, TCSANOW, &tio) < 0) { close(fd); return -1; }

    if (strcmp(g_cfg.mode, "gpio") != 0) {
        struct serial_rs485 rs485conf;
        memset(&rs485conf, 0, sizeof(rs485conf));
        rs485conf.flags |= SER_RS485_ENABLED;
        if (ioctl(fd, TIOCSRS485, &rs485conf) < 0)
            perror("TIOCSRS485");
        else
            fprintf(stderr, "[serial] TIOCSRS485 enabled (kernel auto-direction)\n");
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

/* ===== 防污染：发送前排空 ===== */
static void drain_rx(int fd, int timeout_ms)
{
    tcflush(fd, TCIFLUSH);
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    int pr = poll(&pfd, 1, timeout_ms);
    if (pr > 0) {
        uint8_t junk[64];
        while (read(fd, junk, sizeof(junk)) > 0) { }
        tcflush(fd, TCIFLUSH);
    }
}

/* 返回：0 成功；-1 写失败；-2 超时；-3 帧错；-4 异常响应；-5 设备断开 */
static int modbus_read(int fd, uint8_t slave, uint8_t func,
                       uint16_t addr, uint16_t qty,
                       uint16_t *out, int timeout_ms)
{
    uint8_t req[8];
    req[0] = slave; req[1] = func;
    req[2] = (uint8_t)(addr >> 8); req[3] = (uint8_t)(addr & 0xFF);
    req[4] = (uint8_t)(qty >> 8);  req[5] = (uint8_t)(qty & 0xFF);
    uint16_t crc = crc16_modbus(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);

    drain_rx(fd, g_cfg.inter_frame_delay_ms);

    rs485_tx();
    ssize_t n = write(fd, req, sizeof(req));
    if (n != (ssize_t)sizeof(req)) {
        rs485_rx();
        if (errno == EIO || errno == ENXIO || errno == ENODEV)
            return -5;          /* 写的时候设备已经没了 */
        return -1;
    }
    tcdrain(fd);
    usleep(g_cfg.post_tx_delay_us);
    rs485_rx();

    uint8_t resp[MAX_RESP_LEN];
    size_t  got = 0, want = 5;
    int     slide = 0;
    int64_t deadline = now_mono_us() + (int64_t)timeout_ms * 1000;

    while (got < want) {
        int64_t remain = deadline - now_mono_us();
        if (remain <= 0) { tcflush(fd, TCIFLUSH); return -2; }

        struct pollfd pfd = { .fd = fd, .events = POLLIN };
        int pr = poll(&pfd, 1, (int)(remain / 1000));
        if (pr < 0) {
            if (errno == EINTR) continue;
            tcflush(fd, TCIFLUSH);
            return -5;          /* poll 出错，按断线处理 */
        }
        if (pr == 0) { tcflush(fd, TCIFLUSH); return -2; }

        /* ① poll 说可读，但队列为 0 → 断线/hangup */
        int pending = 0;
        if (ioctl(fd, FIONREAD, &pending) == 0 && pending == 0) {
            tcflush(fd, TCIFLUSH);
            return -5;
        }

        ssize_t r = read(fd, resp + got, sizeof(resp) - got);

        /* ② read 的返回值区分“暂时没数据”和“设备没了” */
        if (r < 0) {
            if (errno == EIO || errno == ENXIO || errno == ENODEV) {
                tcflush(fd, TCIFLUSH);
                return -5;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                continue;
            tcflush(fd, TCIFLUSH);
            return -5;
        }
        if (r == 0) {
            tcflush(fd, TCIFLUSH);
            return -5;          /* 串口 EOF/hangup */
        }
        got += (size_t)r;

        if (got >= sizeof(req) && memcmp(resp, req, sizeof(req)) == 0) {
            memmove(resp, resp + sizeof(req), got - sizeof(req));
            got -= sizeof(req);
            continue;
        }

        while (got >= 2) {
            uint8_t fc = resp[1];
            int fc_ok = (fc == func) || (fc == (uint8_t)(func | 0x80));
            if (resp[0] == slave && fc_ok) break;
            memmove(resp, resp + 1, got - 1);
            got -= 1;
            if (++slide > MAX_SLIDE_BYTES) { tcflush(fd, TCIFLUSH); return -3; }
        }

        if (got >= 2) {
            if (resp[1] & 0x80) want = 5;
            else if (got >= 3) {
                size_t body = (size_t)resp[2];
                size_t w = 3 + body + 2;
                if (w > sizeof(resp)) {
                    memmove(resp, resp + 1, got - 1);
                    got -= 1;
                    if (++slide > MAX_SLIDE_BYTES) { tcflush(fd, TCIFLUSH); return -3; }
                    want = 5;
                    continue;
                }
                want = w;
            }
        }
    }

    if (resp[0] != slave) return -3;
    uint16_t rxcrc = (uint16_t)resp[got - 2] | ((uint16_t)resp[got - 1] << 8);
    if (rxcrc != crc16_modbus(resp, got - 2)) return -3;
    if (resp[1] & 0x80) return -4;
    if (resp[1] != func) return -3;
    if (resp[2] != qty * 2) return -3;

    for (uint16_t i = 0; i < qty; i++)
        out[i] = ((uint16_t)resp[3 + i * 2] << 8) | resp[4 + i * 2];
    return 0;
}

static double decode_value(const point_cfg_t *p, uint16_t raw)
{
    double v;
    if (p->type == PT_INT16)       v = (int16_t)raw;
    else if (p->type == PT_UINT16) v = (uint16_t)raw;
    else                           v = raw;
    return v * p->scale;
}

static void apply_group(int gi, const uint16_t *raw, int ok)
{
    int64_t mono = now_mono_us();
    int64_t utc  = now_utc_us();
    poll_group_t *g = &g_groups[gi];

    pthread_mutex_lock(&g_mu);
    for (int k = 0; k < g->point_cnt; k++) {
        int pi = g->idx[k];
        point_value_t *pv = &g_snap.points[pi];
        const point_cfg_t *pc = &g_cfg.points[pi];
        if (!ok) { pv->quality = Q_BAD; continue; }
        double v = decode_value(pc, raw[k]);
        pv->value = v;
        if (pc->max > pc->min && (v < pc->min || v > pc->max))
            pv->quality = Q_UNCERTAIN;
        else
            pv->quality = Q_GOOD;
        pv->ts_mono_us = mono;
        pv->ts_utc_us  = utc;
        pv->seq++;
    }
    pthread_mutex_unlock(&g_mu);
}

static void *acq_thread_main(void *arg)
{
    (void)arg;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (g_run) {
        for (int gi = 0; gi < g_group_cnt; gi++) {
            uint16_t raw[MAX_GROUP_POINTS] = {0};
            int ret = -1;

            for (int t = 0; t <= g_cfg.retries; t++) {
                if (t > 0) {
                    tcflush(g_fd, TCIFLUSH);
                    usleep(g_cfg.retry_backoff_ms * 1000);
                    tcflush(g_fd, TCIFLUSH);
                }
                ret = modbus_read(g_fd, g_cfg.slave_id, g_cfg.function,
                                  g_groups[gi].start, g_groups[gi].count,
                                  raw, g_cfg.timeout_ms);
                if (ret == 0) break;
            }

            apply_group(gi, raw, ret == 0);

            pthread_mutex_lock(&g_mu);
            if (ret == 0) {
                g_snap.dev_seq++;
                g_snap.fail_count = 0;
                g_snap.last_good_us = now_mono_us();
            } else {
                g_snap.fail_count++;
            }
            pthread_mutex_unlock(&g_mu);

            if (ret != 0)
                fprintf(stderr, "[acq] group 0x%04X failed ret=%d\n",
                        g_groups[gi].start, ret);

            if (g_cfg.inter_frame_delay_ms > 0) {
                tcflush(g_fd, TCIFLUSH);
                usleep(g_cfg.inter_frame_delay_ms * 1000);
                tcflush(g_fd, TCIFLUSH);
            }
        }

        /* 心跳：每轮结束后更新，监控线程据此判断是否卡死 */
        atomic_store(&g_heartbeat_us, now_mono_us());

        next.tv_nsec += g_cfg.poll_interval_ms * 1000000L;
        while (next.tv_nsec >= 1000000000L) {
            next.tv_nsec -= 1000000000L;
            next.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }
    return NULL;
}

int acq_thread_start(const app_config_t *cfg)
{
    g_cfg = *cfg;
    memset(&g_snap, 0, sizeof(g_snap));
    g_snap.point_count = g_cfg.point_count;
    for (int i = 0; i < g_cfg.point_count; i++) {
        g_snap.points[i].quality = Q_STALE;
        g_snap.points[i].ts_mono_us = now_mono_us();
    }
    build_groups();
    if (rs485_init() < 0) return -1;
    g_fd = serial_open();
    if (g_fd < 0) return -1;

    atomic_store(&g_heartbeat_us, now_mono_us());
    g_run = 1;
    if (pthread_create(&g_tid, NULL, acq_thread_main, NULL) != 0) {
        g_run = 0; close(g_fd); g_fd = -1;
        return -1;
    }
    return 0;
}

void acq_thread_stop(void)
{
    g_run = 0;
    pthread_join(g_tid, NULL);
    if (g_fd >= 0) { close(g_fd); g_fd = -1; }
    if (g_de)   gpiod_line_release(g_de);
    if (g_chip) gpiod_chip_close(g_chip);
}

void acq_snapshot_get(acq_snapshot_t *out)
{
    int64_t now = now_mono_us();
    pthread_mutex_lock(&g_mu);
    *out = g_snap;
    pthread_mutex_unlock(&g_mu);
    for (int i = 0; i < out->point_count; i++) {
        point_value_t *p = &out->points[i];
        if (p->quality == Q_GOOD &&
            (now - p->ts_mono_us) > (int64_t)g_cfg.stale_after_ms * 1000)
            p->quality = Q_STALE;
    }
}

long long acq_heartbeat_age_ms(void)
{
    long long hb = atomic_load(&g_heartbeat_us);
    if (hb == 0) return -1;
    return (now_mono_us() - hb) / 1000;
}