#define _GNU_SOURCE

#include "can_acq.h"
#include "log.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define CAN_RX_BATCH       32
#define CAN_REOPEN_MS      5000
#define CAN_CMD_REPEAT_US  (5 * 1000000LL)

typedef struct {
    uint32_t  fault;
    int       alarm_level;
    int       comm_lost;
    int64_t   last_rx_us;
    int64_t   last_cmd_us;
    can_cmd_t last_cmd;
    double    last_cmd_param;
    uint32_t  cmd_seq;
} can_node_rt_t;

static app_config_t    g_cfg;
static int             g_fd = -1;
static pthread_t       g_tid;
static volatile int    g_run = 0;
static pthread_mutex_t g_mu    = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_tx_mu = PTHREAD_MUTEX_INITIALIZER;

static can_snapshot_t  g_snap;
static can_node_rt_t   g_rt[MAX_CAN_NODES];
static atomic_llong    g_heartbeat_us = 0;

static int     g_open_warned  = 0;
static int64_t g_next_open_us = 0;

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

const char *can_cmd_name(can_cmd_t cmd)
{
    switch (cmd) {
    case CAN_CMD_REDUCE_LOAD: return "reduce_load";
    case CAN_CMD_STOP:        return "stop";
    case CAN_CMD_BOOST:       return "boost";
    case CAN_CMD_CLEAR_FAULT: return "clear_fault";
    default:                  return "none";
    }
}

static uint16_t u16_of(const uint8_t *p, int le)
{
    if (le)
        return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static uint32_t u32_of(const uint8_t *p, int le)
{
    if (le)
        return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
               ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

/* 按 signal.little_endian 解析；多字节默认大端，float 为 IEEE754 4 字节 */
static int decode_signal(const can_signal_cfg_t *s, const struct can_frame *f, double *out)
{
    if ((int)s->start_byte + (int)s->len > 8) return -1;
    if ((int)s->start_byte + (int)s->len > (int)f->can_dlc) return -1;

    const uint8_t *p = &f->data[s->start_byte];
    double raw;

    switch (s->type) {
    case PT_INT16:
        if (s->len == 1)      raw = (double)(int8_t)p[0];
        else if (s->len == 2) raw = (double)(int16_t)u16_of(p, s->little_endian);
        else {
            uint32_t u = u32_of(p, s->little_endian);
            int32_t v;
            memcpy(&v, &u, sizeof(v));
            raw = (double)v;
        }
        break;

    case PT_UINT16:
        if (s->len == 1)      raw = (double)p[0];
        else if (s->len == 2) raw = (double)u16_of(p, s->little_endian);
        else                  raw = (double)u32_of(p, s->little_endian);
        break;

    case PT_INT32:
        if (s->len == 1)      raw = (double)(int8_t)p[0];
        else if (s->len == 2) raw = (double)(int16_t)u16_of(p, s->little_endian);
        else {
            uint32_t u = u32_of(p, s->little_endian);
            int32_t v;
            memcpy(&v, &u, sizeof(v));
            raw = (double)v;
        }
        break;

    case PT_UINT32:
        if (s->len == 1)      raw = (double)p[0];
        else if (s->len == 2) raw = (double)u16_of(p, s->little_endian);
        else                  raw = (double)u32_of(p, s->little_endian);
        break;

    case PT_FLOAT:
        if (s->len != 4) return -1;
        {
            uint32_t u = u32_of(p, s->little_endian);
            float v;
            memcpy(&v, &u, sizeof(v));
            raw = (double)v;
        }
        break;

    default:
        return -1;
    }

    *out = raw * s->scale + s->offset;
    return 0;
}

static int frame_priority(uint32_t can_id, uint32_t *node_mask)
{
    int high = 0;
    *node_mask = 0;

    for (int i = 0; i < g_cfg.can.signal_count; i++) {
        const can_signal_cfg_t *s = &g_cfg.can.signals[i];
        if (s->resp_id != can_id) continue;
        *node_mask |= (1u << s->node);
        if (s->priority) high = 1;
    }
    return high;
}

static void set_signal_locked(int si, int ok, double value,
                              int64_t mono_us, int64_t utc_us)
{
    can_signal_value_t *sv = &g_snap.signals[si];
    const can_signal_cfg_t *s = &g_cfg.can.signals[si];

    sv->ts_mono_us = mono_us;
    sv->ts_utc_us  = utc_us;

    if (!ok) {
        sv->quality = Q_BAD;
        return;
    }

    sv->value = value;
    if (s->min < s->max && (value < s->min || value > s->max))
        sv->quality = Q_UNCERTAIN;
    else
        sv->quality = Q_GOOD;

    sv->seq++;
}

static void reason_add(char *buf, size_t len, size_t *off, const char *fmt, ...)
{
    if (*off >= len - 1) return;

    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(buf + *off, len - *off, fmt, ap);
    va_end(ap);

    if (w > 0) {
        *off += (size_t)w;
        if (*off >= len) *off = len - 1;
    }
}

/*
 * 电压/电流保护判断。high 帧一收到就调用；每轮结束也会再刷新一次，
 * 这样持续故障时命令可以按 5s 周期重发。
 */
static void eval_and_maybe_send(int ni, int64_t now)
{
    uint32_t fault = 0;
    int      level = 0;
    char     reason[128];
    size_t   off = 0;
    int      changed;

    reason[0] = '\0';

    pthread_mutex_lock(&g_mu);

    for (int si = 0; si < g_cfg.can.signal_count; si++) {
        const can_signal_cfg_t *s = &g_cfg.can.signals[si];
        if (s->node != ni) continue;
        if (s->role != CAN_ROLE_VOLTAGE && s->role != CAN_ROLE_CURRENT) continue;

        const can_signal_value_t *sv = &g_snap.signals[si];
        if (sv->quality == Q_BAD || sv->quality == Q_STALE) continue;

        double v    = sv->value;
        int    is_v = (s->role == CAN_ROLE_VOLTAGE);

        if (s->crit_hi > 0 && v >= s->crit_hi) {
            fault |= is_v ? CAN_FAULT_OVERVOLTAGE : CAN_FAULT_OVERCURRENT;
            level = 2;
            reason_add(reason, sizeof(reason), &off, "%s=%.2f>=%.2f; ", s->name, v, s->crit_hi);
        } else if (s->warn_hi > 0 && v >= s->warn_hi) {
            fault |= is_v ? CAN_FAULT_OVERVOLTAGE : CAN_FAULT_OVERCURRENT;
            if (level < 1) level = 1;
            reason_add(reason, sizeof(reason), &off, "%s=%.2f>%.2f; ", s->name, v, s->warn_hi);
        }

        if (s->crit_lo > 0 && v <= s->crit_lo) {
            if (is_v) fault |= CAN_FAULT_UNDERVOLTAGE;
            level = 2;
            reason_add(reason, sizeof(reason), &off, "%s=%.2f<=%.2f; ", s->name, v, s->crit_lo);
        } else if (s->warn_lo > 0 && v <= s->warn_lo) {
            if (is_v) fault |= CAN_FAULT_UNDERVOLTAGE;
            if (level < 1) level = 1;
            reason_add(reason, sizeof(reason), &off, "%s=%.2f<%.2f; ", s->name, v, s->warn_lo);
        }
    }

    if (g_rt[ni].comm_lost) {
        fault |= CAN_FAULT_COMM_LOST;
        if (level < 2) level = 2;
        reason_add(reason, sizeof(reason), &off, "comm_lost; ");
    }

    uint32_t old_fault = g_rt[ni].fault;
    int      old_level = g_rt[ni].alarm_level;

    changed = (fault != old_fault) || (level != old_level);

    g_rt[ni].fault       = fault;
    g_rt[ni].alarm_level = level;
    g_snap.nodes[ni].fault       = fault;
    g_snap.nodes[ni].alarm_level = level;

    can_cmd_t cmd = CAN_CMD_NONE;
    int need_cmd  = 0;

    if (fault & (CAN_FAULT_OVERVOLTAGE | CAN_FAULT_UNDERVOLTAGE | CAN_FAULT_OVERCURRENT)) {
        cmd = (level >= 2) ? CAN_CMD_STOP : CAN_CMD_REDUCE_LOAD;
        need_cmd = changed || (now - g_rt[ni].last_cmd_us >= CAN_CMD_REPEAT_US);
    } else if (!fault && old_fault &&
               (old_fault & (CAN_FAULT_OVERVOLTAGE | CAN_FAULT_UNDERVOLTAGE |
                             CAN_FAULT_OVERCURRENT))) {
        /* 电压/电流保护恢复：通知从机清故障/恢复运行 */
        cmd = CAN_CMD_CLEAR_FAULT;
        need_cmd = 1;
    }

    pthread_mutex_unlock(&g_mu);

    if (fault && fault != old_fault)
        LOG_W("can", "node %s fault=0x%02X level=%d (%s)",
              g_cfg.can.nodes[ni].name, fault, level, reason);
    else if (!fault && old_fault)
        LOG_I("can", "node %s recovered", g_cfg.can.nodes[ni].name);

    if (need_cmd && cmd != CAN_CMD_NONE) {
        double     param = (cmd == CAN_CMD_CLEAR_FAULT) ? 0.0
                          : ((level >= 2) ? 100.0 : 50.0);
        const char *why  = (cmd == CAN_CMD_CLEAR_FAULT)
                           ? "voltage/current protection cleared"
                           : (reason[0] ? reason : "voltage/current protection");
        can_cmd_send(ni, cmd, param, why);
    }
}

static void handle_frame(const struct can_frame *f, int high, uint32_t node_mask)
{
    int64_t mono = now_mono_us();
    int64_t utc  = now_utc_us();
    int     bad  = 0;

    pthread_mutex_lock(&g_mu);

    for (int i = 0; i < g_cfg.can.signal_count; i++) {
        const can_signal_cfg_t *s = &g_cfg.can.signals[i];
        if (s->resp_id != f->can_id) continue;

        double value = 0.0;
        int ok = (decode_signal(s, f, &value) == 0);
        set_signal_locked(i, ok, value, mono, utc);
        if (!ok) bad++;
    }

    g_snap.rx_frames++;
    if (high) g_snap.rx_high++;
    else      g_snap.rx_normal++;
    g_snap.last_rx_us = mono;

    for (int ni = 0; ni < g_snap.node_count; ni++) {
        if (!(node_mask & (1u << ni))) continue;
        g_rt[ni].last_rx_us          = mono;
        g_rt[ni].comm_lost           = 0;
        g_snap.nodes[ni].last_rx_us  = mono;
        g_snap.nodes[ni].rx_frames++;
    }

    pthread_mutex_unlock(&g_mu);

    if (bad)
        LOG_W("can", "frame id=0x%03X decode failed (dlc=%u)", f->can_id, f->can_dlc);

    if (high) {
        for (int ni = 0; ni < g_snap.node_count; ni++)
            if (node_mask & (1u << ni))
                eval_and_maybe_send(ni, mono);
    }
}

static void mark_comm(int64_t now)
{
    int64_t stale_us = (int64_t)g_cfg.can.stale_after_ms * 1000;
    int64_t lost_us  = stale_us * 2;

    pthread_mutex_lock(&g_mu);

    for (int i = 0; i < g_snap.signal_count; i++) {
        can_signal_value_t *sv = &g_snap.signals[i];
        if (sv->quality == Q_GOOD && now - sv->ts_mono_us > stale_us)
            sv->quality = Q_STALE;
    }

    for (int ni = 0; ni < g_snap.node_count; ni++) {
        int lost = (!g_snap.link_up) || (now - g_rt[ni].last_rx_us > lost_us);
        if (lost == g_rt[ni].comm_lost) continue;

        g_rt[ni].comm_lost = lost;
        if (lost) {
            g_rt[ni].fault |= CAN_FAULT_COMM_LOST;
            if (g_rt[ni].alarm_level < 2) g_rt[ni].alarm_level = 2;
            LOG_W("can", "node %s comm lost (no frame for %lld ms)",
                  g_cfg.can.nodes[ni].name, (long long)((now - g_rt[ni].last_rx_us) / 1000));
        } else {
            g_rt[ni].fault &= ~CAN_FAULT_COMM_LOST;
            LOG_I("can", "node %s comm recovered", g_cfg.can.nodes[ni].name);
        }

        g_snap.nodes[ni].fault       = g_rt[ni].fault;
        g_snap.nodes[ni].alarm_level = g_rt[ni].alarm_level;
    }

    pthread_mutex_unlock(&g_mu);
}

static void close_link(const char *op, int err)
{
    pthread_mutex_lock(&g_tx_mu);
    int fd = g_fd;
    g_fd = -1;
    if (fd >= 0) close(fd);
    pthread_mutex_unlock(&g_tx_mu);

    g_next_open_us = now_mono_us() + (int64_t)CAN_REOPEN_MS * 1000;

    pthread_mutex_lock(&g_mu);
    g_snap.link_up = 0;
    pthread_mutex_unlock(&g_mu);

    if (err)
        LOG_W("can", "link down (%s): %s", op, strerror(err));
    else
        LOG_W("can", "link down (%s)", op);
}

static int can_open(void)
{
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) return -1;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", g_cfg.can.device);

    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        int err = errno;
        close(fd);
        errno = err;
        return -1;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        int err = errno;
        close(fd);
        errno = err;
        return -1;
    }

    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    /* 只收配置里出现过的响应 ID，减少总线噪声 */
    struct can_filter filters[MAX_CAN_SIGNALS];
    int nf = 0;

    for (int i = 0; i < g_cfg.can.signal_count; i++) {
        uint32_t id = g_cfg.can.signals[i].resp_id;
        if (id == 0) continue;

        int dup = 0;
        for (int k = 0; k < nf; k++)
            if (filters[k].can_id == id) { dup = 1; break; }
        if (dup) continue;

        filters[nf].can_id   = id & CAN_SFF_MASK;
        filters[nf].can_mask = CAN_SFF_MASK;
        nf++;
    }

    if (nf > 0) {
        if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER,
                       filters, (socklen_t)(nf * sizeof(filters[0]))) < 0)
            LOG_W("can", "set filter failed: %s", strerror(errno));
    }

    return fd;
}

static int send_request(int si)
{
    if (g_fd < 0) return -1;

    const can_signal_cfg_t *s = &g_cfg.can.signals[si];

    struct can_frame f;
    memset(&f, 0, sizeof(f));
    f.can_id  = s->req_id & CAN_SFF_MASK;
    f.can_dlc = 8;
    f.data[0] = (uint8_t)(g_cfg.can.nodes[s->node].node_id & 0xFF);
    f.data[1] = (uint8_t)s->role;
    f.data[2] = s->start_byte;
    f.data[3] = s->len;
    /* data[4..7] 保留，实际从机协议确定后在这里补 */

    ssize_t n = write(g_fd, &f, sizeof(f));
    if (n != (ssize_t)sizeof(f)) {
        int err = errno;
        if (err == EAGAIN || err == EWOULDBLOCK) return -1;
        close_link("request write", err);
        return -1;
    }

    pthread_mutex_lock(&g_mu);
    g_snap.tx_requests++;
    pthread_mutex_unlock(&g_mu);
    return 0;
}

static void wait_frames(int64_t deadline_us)
{
    while (g_run && g_fd >= 0) {
        int64_t now = now_mono_us();
        if (now >= deadline_us) break;

        int tmo = (int)((deadline_us - now) / 1000);
        if (tmo < 1) tmo = 1;
        if (tmo > 200) tmo = 200;   /* 让 stop 能及时返回 */

        struct pollfd pfd = { .fd = g_fd, .events = POLLIN };
        int pr = poll(&pfd, 1, tmo);
        if (pr < 0) {
            if (errno == EINTR) continue;
            close_link("poll", errno);
            break;
        }
        if (pr == 0) continue;
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            close_link("poll revents", 0);
            break;
        }
        if (!(pfd.revents & POLLIN)) continue;

        struct can_frame frames[CAN_RX_BATCH];
        int n = 0;

        while (n < CAN_RX_BATCH) {
            struct can_frame f;
            ssize_t r = read(g_fd, &f, sizeof(f));

            if (r < 0) {
                int err = errno;
                if (err == EAGAIN || err == EWOULDBLOCK) break;
                if (err == EINTR) continue;
                close_link("read", err);
                break;
            }
            if (r != (ssize_t)sizeof(f)) break;
            if (f.can_id & CAN_ERR_FLAG) continue;
            frames[n++] = f;
        }

        if (g_fd < 0) break;
        if (n == 0) continue;

        int      high[CAN_RX_BATCH];
        uint32_t mask[CAN_RX_BATCH];
        for (int k = 0; k < n; k++)
            high[k] = frame_priority(frames[k].can_id, &mask[k]);

        /* 优先级：先处理 high（电压/电流），再处理 normal（流量） */
        for (int pass = 1; pass >= 0; pass--) {
            for (int k = 0; k < n; k++) {
                if (high[k] != pass || mask[k] == 0) continue;
                handle_frame(&frames[k], high[k], mask[k]);
            }
        }
    }
}

static void run_cycle(void)
{
    int64_t  now = now_mono_us();
    uint32_t base[MAX_CAN_SIGNALS];

    mark_comm(now);

    pthread_mutex_lock(&g_mu);
    for (int i = 0; i < g_snap.signal_count; i++)
        base[i] = g_snap.signals[i].seq;
    pthread_mutex_unlock(&g_mu);

    int any_request = 0;
    for (int i = 0; i < g_cfg.can.signal_count; i++)
        if (g_cfg.can.signals[i].req_id) { any_request = 1; break; }

    for (int attempt = 0; attempt <= g_cfg.can.retries; attempt++) {
        int      sent = 0;
        uint32_t sent_req[MAX_CAN_SIGNALS];
        int      sent_req_n = 0;

        /* 高优先级先发：电压、电流 */
        for (int pass = 1; pass >= 0; pass--) {
            for (int i = 0; i < g_cfg.can.signal_count; i++) {
                const can_signal_cfg_t *s = &g_cfg.can.signals[i];
                if (!s->req_id || s->priority != pass) continue;

                int got;
                pthread_mutex_lock(&g_mu);
                got = (g_snap.signals[i].seq != base[i]);
                pthread_mutex_unlock(&g_mu);
                if (attempt > 0 && got) continue;

                /* 多个信号可共用同一个请求帧，按 req_id 去重 */
                int dup = 0;
                for (int k = 0; k < sent_req_n; k++) {
                    if (sent_req[k] == s->req_id) { dup = 1; break; }
                }
                if (dup) continue;

                if (send_request(i) == 0) {
                    sent++;
                    if (sent_req_n < MAX_CAN_SIGNALS)
                        sent_req[sent_req_n++] = s->req_id;
                }
            }
        }

        int64_t deadline;
        if (sent == 0) {
            if (attempt > 0) break;
            /* 全部是被动信号时，留一个采集周期收周期帧 */
            deadline = now_mono_us() + (int64_t)g_cfg.can.poll_interval_ms * 1000;
        } else {
            deadline = now_mono_us() + (int64_t)g_cfg.can.response_timeout_ms * 1000;
        }

        wait_frames(deadline);

        if (!any_request) break;

        int missing = 0;
        pthread_mutex_lock(&g_mu);
        for (int i = 0; i < g_cfg.can.signal_count; i++)
            if (g_cfg.can.signals[i].req_id && g_snap.signals[i].seq == base[i])
                missing++;
        pthread_mutex_unlock(&g_mu);

        if (missing == 0) break;
        if (attempt < g_cfg.can.retries) usleep(2000);
    }

    /* 请求了但没收到响应：记超时，质量由 stale 逻辑降级 */
    pthread_mutex_lock(&g_mu);
    for (int i = 0; i < g_cfg.can.signal_count; i++) {
        if (!g_cfg.can.signals[i].req_id) continue;
        if (g_snap.signals[i].seq == base[i]) {
            g_snap.signals[i].timeouts++;
            g_snap.tx_timeouts++;
        }
    }
    pthread_mutex_unlock(&g_mu);

    now = now_mono_us();
    mark_comm(now);

    /* 持续故障时按 5s 重发保护命令；comm lost 恢复后清除报警级别 */
    for (int ni = 0; ni < g_snap.node_count; ni++)
        eval_and_maybe_send(ni, now);

    pthread_mutex_lock(&g_mu);
    g_snap.seq++;
    pthread_mutex_unlock(&g_mu);
}

static void *can_thread_main(void *arg)
{
    (void)arg;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (g_run) {
        if (g_fd < 0 && now_mono_us() >= g_next_open_us) {
            int fd = can_open();
            if (fd >= 0) {
                pthread_mutex_lock(&g_tx_mu);
                g_fd = fd;
                pthread_mutex_unlock(&g_tx_mu);
                pthread_mutex_lock(&g_mu);
                g_snap.link_up = 1;
                pthread_mutex_unlock(&g_mu);
                LOG_I("can", "%s opened: bitrate=%d nodes=%d signals=%d",
                      g_cfg.can.device, g_cfg.can.bitrate,
                      g_cfg.can.node_count, g_cfg.can.signal_count);
                g_open_warned = 0;
            } else {
                int err = errno;
                pthread_mutex_lock(&g_mu);
                g_snap.link_up = 0;
                pthread_mutex_unlock(&g_mu);
                if (!g_open_warned) {
                    LOG_W("can", "open %s failed: %s, retry every %dms",
                          g_cfg.can.device, strerror(err), CAN_REOPEN_MS);
                    g_open_warned = 1;
                }
                g_next_open_us = now_mono_us() + (int64_t)CAN_REOPEN_MS * 1000;
            }
        }

        if (g_fd >= 0) run_cycle();
        else           mark_comm(now_mono_us());

        atomic_store(&g_heartbeat_us, now_mono_us());

        next.tv_nsec += (long)g_cfg.can.poll_interval_ms * 1000000L;
        while (next.tv_nsec >= 1000000000L) {
            next.tv_nsec -= 1000000000L;
            next.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }
    return NULL;
}

int can_acq_start(const app_config_t *cfg)
{
    g_cfg = *cfg;
    memset(&g_snap, 0, sizeof(g_snap));
    memset(g_rt, 0, sizeof(g_rt));

    g_fd          = -1;
    g_open_warned = 0;
    g_next_open_us = 0;
    g_run         = 0;

    g_snap.enabled      = g_cfg.can.enabled;
    g_snap.node_count   = g_cfg.can.enabled ? g_cfg.can.node_count : 0;
    g_snap.signal_count = g_cfg.can.enabled ? g_cfg.can.signal_count : 0;

    int64_t now = now_mono_us();
    int64_t utc = now_utc_us();

    for (int i = 0; i < g_snap.node_count; i++) {
        snprintf(g_snap.nodes[i].name, sizeof(g_snap.nodes[i].name),
                 "%s", g_cfg.can.nodes[i].name);
        g_snap.nodes[i].node_id    = g_cfg.can.nodes[i].node_id;
        g_snap.nodes[i].last_rx_us = now;
        g_rt[i].last_rx_us         = now;
    }

    for (int i = 0; i < g_snap.signal_count; i++) {
        g_snap.signals[i].node      = g_cfg.can.signals[i].node;
        g_snap.signals[i].priority  = g_cfg.can.signals[i].priority;
        g_snap.signals[i].quality   = Q_STALE;
        g_snap.signals[i].ts_mono_us = now;
        g_snap.signals[i].ts_utc_us  = utc;
    }

    if (!g_cfg.can.enabled) {
        LOG_I("can", "disabled");
        return 0;
    }
    if (g_snap.signal_count == 0) {
        LOG_W("can", "enabled but no valid signal, skipped");
        g_snap.enabled      = 0;
        g_snap.node_count   = 0;
        g_snap.signal_count = 0;
        return 0;
    }

    g_run = 1;
    if (pthread_create(&g_tid, NULL, can_thread_main, NULL) != 0) {
        g_run = 0;
        g_snap.enabled = 0;
        return -1;
    }
    return 0;
}

void can_acq_stop(void)
{
    if (g_run) {
        g_run = 0;
        pthread_join(g_tid, NULL);
    }
    pthread_mutex_lock(&g_tx_mu);
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
    pthread_mutex_unlock(&g_tx_mu);
    pthread_mutex_lock(&g_mu);
    g_snap.link_up = 0;
    pthread_mutex_unlock(&g_mu);
}

void can_acq_snapshot_get(can_snapshot_t *out)
{
    pthread_mutex_lock(&g_mu);
    *out = g_snap;
    pthread_mutex_unlock(&g_mu);

    if (!out->enabled) {
        out->node_count   = 0;
        out->signal_count = 0;
        return;
    }

    int64_t now     = now_mono_us();
    int64_t stale   = (int64_t)g_cfg.can.stale_after_ms * 1000;
    int64_t lost_us = stale * 2;

    for (int i = 0; i < out->signal_count; i++) {
        can_signal_value_t *sv = &out->signals[i];
        if (sv->quality == Q_GOOD && now - sv->ts_mono_us > stale)
            sv->quality = Q_STALE;
    }

    for (int ni = 0; ni < out->node_count; ni++) {
        int lost = (!out->link_up) || (now - out->nodes[ni].last_rx_us > lost_us);
        if (!lost) continue;
        out->nodes[ni].fault |= CAN_FAULT_COMM_LOST;
        if (out->nodes[ni].alarm_level < 2)
            out->nodes[ni].alarm_level = 2;
    }
}

long long can_acq_heartbeat_age_ms(void)
{
    long long hb = atomic_load(&g_heartbeat_us);
    if (hb == 0) return -1;
    return (now_mono_us() - hb) / 1000;
}

int can_cmd_send(int node, can_cmd_t cmd, double param, const char *reason)
{
    if (node < 0 || node >= g_cfg.can.node_count) return -1;

    const can_node_cfg_t *n = &g_cfg.can.nodes[node];
    if (n->cmd_id == 0) {
        LOG_W("can", "node %s has no cmd_id", n->name);
        return -1;
    }

    int p10 = (int)(param * 10.0 + (param >= 0 ? 0.5 : -0.5));
    if (p10 < 0)     p10 = 0;
    if (p10 > 65535) p10 = 65535;

    struct can_frame f;
    memset(&f, 0, sizeof(f));
    f.can_id  = n->cmd_id & CAN_SFF_MASK;
    f.can_dlc = 3;
    f.data[0] = (uint8_t)cmd;
    f.data[1] = (uint8_t)(p10 >> 8);
    f.data[2] = (uint8_t)(p10 & 0xFF);

    int sent = 0;

    pthread_mutex_lock(&g_tx_mu);

    if (!g_cfg.can.tx_enabled) {
        /* 硬件未接：只打印，不写总线 */
        LOG_I("can", "TX(print-only) node=%s id=0x%03X cmd=%s param=%.1f "
                     "reason=%s data=[%02X %02X %02X]",
              n->name, n->cmd_id, can_cmd_name(cmd), param,
              reason ? reason : "-", f.data[0], f.data[1], f.data[2]);
        fprintf(stderr, "[can] TX(print-only) node=%s cmd=%s param=%.1f\n",
                n->name, can_cmd_name(cmd), param);
        sent = 1;
    } else if (g_fd < 0) {
        LOG_W("can", "TX dropped: %s link down, node=%s cmd=%s",
              g_cfg.can.device, n->name, can_cmd_name(cmd));
    } else {
        ssize_t w = write(g_fd, &f, sizeof(f));
        if (w == (ssize_t)sizeof(f)) {
            LOG_I("can", "TX node=%s id=0x%03X cmd=%s param=%.1f reason=%s",
                  n->name, n->cmd_id, can_cmd_name(cmd), param,
                  reason ? reason : "-");
            sent = 1;
        } else {
            LOG_W("can", "TX failed: %s (node=%s cmd=%s)",
                  strerror(errno), n->name, can_cmd_name(cmd));
        }
    }

    if (sent) {
        int64_t now = now_mono_us();
        pthread_mutex_lock(&g_mu);
        g_rt[node].last_cmd       = cmd;
        g_rt[node].last_cmd_param = param;
        g_rt[node].last_cmd_us    = now;
        g_rt[node].cmd_seq++;

        g_snap.nodes[node].cmd_seq        = g_rt[node].cmd_seq;
        g_snap.nodes[node].last_cmd_param = param;
        snprintf(g_snap.nodes[node].last_cmd,
                 sizeof(g_snap.nodes[node].last_cmd),
                 "%s", can_cmd_name(cmd));
        pthread_mutex_unlock(&g_mu);
    }

    pthread_mutex_unlock(&g_tx_mu);
    return sent ? 0 : -1;
}
