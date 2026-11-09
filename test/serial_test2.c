/*
 * serial_rx.c —— 串口接收程序(RK3506B)
 *
 * 编译: gcc serial_rx.c -o serial_rx          (板上直接编译)
 *       aarch64-linux-gnu-gcc serial_rx.c -o serial_rx  (交叉编译)
 * 用法: ./serial_rx /dev/ttyS3 [115200]
 *       ./serial_rx COM2 9600
 */
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <strings.h>    /* strcasecmp */
#include <stdlib.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/select.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <time.h>

#ifndef CMSPAR
#define CMSPAR 010000000000
#endif

#define SERIAL_FLUSH_TX        1
#define SERIAL_FLUSH_RX        2

#define SERIAL_PARITY_NO       0
#define SERIAL_PARITY_ODD      1
#define SERIAL_PARITY_EVENT    2
#define SERIAL_PARITY_MARK     3
#define SERIAL_PARITY_SPACE    4

#define SERIAL_STOPBIT_ONE     1
#define SERIAL_STOPBIT_TWO     2

static int serial_fd = -1;
static struct termios old_tios;
static volatile sig_atomic_t g_running = 1;

static void on_signal(int sig)
{
    (void)sig;
    g_running = 0;
}

/* RK3506B: 一般直接传 /dev/ttySx 即可; 保留 COMx 映射方便老脚本 */
char *get_serial_dev_name(char *serial_name)
{
    if (strcasecmp(serial_name, "COM1") == 0)      return "/dev/ttyS1";
    else if (strcasecmp(serial_name, "COM2") == 0) return "/dev/ttyS2";
    else if (strcasecmp(serial_name, "COM3") == 0) return "/dev/ttyS3";
    else if (strcasecmp(serial_name, "COM4") == 0) return "/dev/ttyS4";
    else if (strcasecmp(serial_name, "COM5") == 0) return "/dev/ttyS5";
    else if (strcasecmp(serial_name, "COM6") == 0) return "/dev/ttyS6";
    else if (strcasecmp(serial_name, "COM7") == 0) return "/dev/ttyS7";
    else if (strcasecmp(serial_name, "COM9") == 0) return "/dev/ttyS9";
    else                                           return serial_name;   /* 直接传设备路径 */
}

static void dump_frame(const unsigned char *buf, int len)
{
    struct timeval tv;
    struct tm tm_now;
    int i;

    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &tm_now);

    printf("[%02d:%02d:%02d.%03ld] RX %d byte(s):",
           tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec,
           (long)(tv.tv_usec / 1000), len);

    for (i = 0; i < len; i++)
        printf(" %02X", buf[i]);

    printf("   |");
    for (i = 0; i < len; i++)
        putchar((buf[i] >= 0x20 && buf[i] <= 0x7E) ? buf[i] : '.');
    printf("|\n");

    fflush(stdout);
}

static int termios_init(struct termios *tios, int baud,
                        int parity, int data_bits, int stop_bits)
{
    speed_t baud_rate;

    if (tios == NULL)
        return -1;

    memset(tios, 0, sizeof(*tios));

    /* ---- 原始模式(RX) ---- */
    tios->c_iflag = 0;              /* 不做输入转换; 若想丢弃校验错字节可加 IGNPAR */
    tios->c_oflag = 0;
    tios->c_lflag = 0;
    tios->c_cflag = CREAD | CLOCAL;

    tios->c_cc[VMIN]  = 1;          /* 至少收到 1 字节才返回 */
    tios->c_cc[VTIME] = 0;

    if (data_bits == 5)      tios->c_cflag |= CS5;
    else if (data_bits == 6) tios->c_cflag |= CS6;
    else if (data_bits == 7) tios->c_cflag |= CS7;
    else if (data_bits == 8) tios->c_cflag |= CS8;
    else                     return -1;

    if (stop_bits == 1)      tios->c_cflag &= ~CSTOPB;
    else if (stop_bits == 2) tios->c_cflag |= CSTOPB;
    else                     return -1;

    if (parity == SERIAL_PARITY_NO) {
        tios->c_cflag &= ~PARENB;
        tios->c_cflag &= ~PARODD;
    } else if (parity == SERIAL_PARITY_EVENT) {
        tios->c_cflag |= PARENB;
        tios->c_cflag &= ~PARODD;
    } else if (parity == SERIAL_PARITY_ODD) {
        tios->c_cflag |= PARENB;
        tios->c_cflag |= PARODD;
    } else if (parity == SERIAL_PARITY_MARK) {
        tios->c_cflag |= PARENB | CMSPAR | PARODD;
    } else if (parity == SERIAL_PARITY_SPACE) {
        tios->c_cflag |= PARENB | CMSPAR;
        tios->c_cflag &= ~PARODD;
    } else {
        return -1;
    }

    switch (baud) {
        case 110:    baud_rate = B110;    break;
        case 300:    baud_rate = B300;    break;
        case 600:    baud_rate = B600;    break;
        case 1200:   baud_rate = B1200;   break;
        case 2400:   baud_rate = B2400;   break;
        case 4800:   baud_rate = B4800;   break;
        case 9600:   baud_rate = B9600;   break;
        case 19200:  baud_rate = B19200;  break;
        case 38400:  baud_rate = B38400;  break;
        case 57600:  baud_rate = B57600;  break;
        case 115200: baud_rate = B115200; break;
        case 230400: baud_rate = B230400; break;
        case 460800: baud_rate = B460800; break;
        case 576000: baud_rate = B576000; break;
        case 921600: baud_rate = B921600; break;
        default:     return -1;
    }

    if (cfsetispeed(tios, baud_rate) < 0 || cfsetospeed(tios, baud_rate) < 0)
        return -1;

    return 0;
}

/* 成功返回 1, 失败返回 0 */
int serial_open(char *serial_name, int baud, int parity,
                int data_bits, int stop_bits, int timeout)
{
    struct termios settings;
    char *serial_dev_name;
    int fd;

    (void)timeout;

    if (serial_fd >= 0)
        return 1;

    if (termios_init(&settings, baud, parity, data_bits, stop_bits) < 0) {
        fprintf(stderr, "termios_init failed (baud=%d)\n", baud);
        return 0;
    }

    serial_dev_name = get_serial_dev_name(serial_name);
    if (serial_dev_name == NULL) {
        fprintf(stderr, "invalid serial name: %s\n", serial_name);
        return 0;
    }

    /* 阻塞打开: 配合 VMIN=1, read 会一直等到有数据 */
    fd = open(serial_dev_name, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "open %s failed: %s\n", serial_dev_name, strerror(errno));
        return 0;
    }

    if (tcgetattr(fd, &old_tios) < 0) {
        fprintf(stderr, "tcgetattr failed: %s\n", strerror(errno));
        close(fd);
        return 0;
    }

    if (tcsetattr(fd, TCSANOW, &settings) < 0) {
        fprintf(stderr, "tcsetattr failed: %s\n", strerror(errno));
        close(fd);
        return 0;
    }

    tcflush(fd, TCIFLUSH);   /* 丢掉打开前残留的数据 */
    serial_fd = fd;
    return 1;
}

int serial_close(void)
{
    if (serial_fd < 0)
        return 0;

    tcsetattr(serial_fd, TCSANOW, &old_tios);
    close(serial_fd);
    serial_fd = -1;
    return 1;
}

int serial_flush(int flag)
{
    if (serial_fd < 0)
        return 0;

    if (flag == SERIAL_FLUSH_TX)
        tcflush(serial_fd, TCOFLUSH);
    else if (flag == SERIAL_FLUSH_RX)
        tcflush(serial_fd, TCIFLUSH);
    else if (flag == (SERIAL_FLUSH_RX | SERIAL_FLUSH_TX))
        tcflush(serial_fd, TCIOFLUSH);

    return 1;
}

/* 返回值: >0 读到字节数, 0 被信号打断, <0 错误 */
int serial_read(char *buf, size_t size)
{
    int n;

    if (serial_fd < 0 || buf == NULL || size == 0)
        return -1;

    n = (int)read(serial_fd, buf, size);
    if (n < 0 && errno == EINTR)
        return 0;
    return n;
}

/* 返回值: >0 有数据可读, 0 超时, <0 错误 */
int serial_poll(int timeout)
{
    fd_set rfds;
    struct timeval tv;

    if (serial_fd < 0)
        return -1;

    FD_ZERO(&rfds);
    FD_SET(serial_fd, &rfds);

    tv.tv_sec  = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;

    return select(serial_fd + 1, &rfds, NULL, NULL, &tv);
}

int main(int argc, char *argv[])
{
    unsigned char rxbuf[1024];
    struct sigaction sa;
    int baud_rate;
    int pret, n;

    if (argc < 2 || argc > 3) {
        fprintf(stderr, "usage: %s <com_port|/dev/ttySx> [baud_rate]\n", argv[0]);
        fprintf(stderr, "  e.g. %s /dev/ttyS3 115200\n", argv[0]);
        return -1;
    }

    baud_rate = (argc == 3) ? atoi(argv[2]) : 115200;
    if (baud_rate <= 0) {
        fprintf(stderr, "invalid baud rate: %s\n", argv[2]);
        return -1;
    }

    /* Ctrl+C 时让阻塞的 select/read 返回 EINTR, 从而优雅退出 */
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    if (serial_open(argv[1], baud_rate, SERIAL_PARITY_NO, 8,
                    SERIAL_STOPBIT_ONE, 1000) == 0) {
        fprintf(stderr, "open serial failed\n");
        return -1;
    }

    printf("opened %s, baud=%d, 8N1, waiting for data... (Ctrl+C to quit)\n",
           argv[1], baud_rate);

    while (g_running) {
        pret = serial_poll(1000);          /* 1s 超时, 便于响应 Ctrl+C */
        if (pret > 0) {
            n = serial_read((char *)rxbuf, sizeof(rxbuf));
            if (n > 0)
                dump_frame(rxbuf, n);
            else if (n < 0 && errno != EAGAIN) {
                fprintf(stderr, "read error: %s\n", strerror(errno));
                break;
            }
        } else if (pret < 0) {
            if (errno == EINTR)
                continue;
            fprintf(stderr, "select error: %s\n", strerror(errno));
            break;
        }
        /* pret == 0: 超时无数据, 继续等 */
    }

    serial_close();
    printf("\nserial closed.\n");
    return 0;
}
