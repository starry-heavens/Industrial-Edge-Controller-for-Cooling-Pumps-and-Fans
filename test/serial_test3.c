/*
 * rs485_modbus_rx.c —— RS485 Modbus RTU 振动传感器读取程序 (RK3506B)
 *
 * 编译: gcc rs485_modbus_rx.c -o rs485_modbus_rx
 * 用法: ./rs485_modbus_rx /dev/ttyS3 [baud_rate]
 *       默认波特率 9600 (请根据传感器实际波特率调整，例如 115200)
 */
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <stdint.h>

// Linux RS485 模式定义 (部分老内核可能没有，手动补全)
#ifndef TIOCSRS485
#define TIOCSRS485 0x542F
#endif
#ifndef SER_RS485_ENABLED
#define SER_RS485_ENABLED (1 << 0)
#endif
struct serial_rs485 {
    uint32_t flags;
    uint32_t delay_rts_before_send;
    uint32_t delay_rts_after_send;
    uint32_t padding[5];
};

static int serial_fd = -1;
static struct termios old_tios;
static volatile sig_atomic_t g_running = 1;

static void on_signal(int sig)
{
    (void)sig;
    g_running = 0;
}

/* Modbus CRC16 计算 */
static uint16_t modbus_crc16(uint8_t *buffer, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < length; i++) {
        crc ^= buffer[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static int termios_init(struct termios *tios, int baud)
{
    speed_t baud_rate;

    memset(tios, 0, sizeof(*tios));
    tios->c_cflag = CREAD | CLOCAL | CS8;  // 8N1
    tios->c_iflag = 0;
    tios->c_oflag = 0;
    tios->c_lflag = 0;
    tios->c_cc[VMIN]  = 0;  // 配合 select 使用
    tios->c_cc[VTIME] = 0;

    switch (baud) {
        case 9600:   baud_rate = B9600;   break;
        case 19200:  baud_rate = B19200;  break;
        case 38400:  baud_rate = B38400;  break;
        case 57600:  baud_rate = B57600;  break;
        case 115200: baud_rate = B115200; break;
        default:     return -1;
    }

    if (cfsetispeed(tios, baud_rate) < 0 || cfsetospeed(tios, baud_rate) < 0)
        return -1;

    return 0;
}

int serial_open(char *serial_name, int baud)
{
    struct termios settings;
    int fd;

    if (termios_init(&settings, baud) < 0) return 0;

    // 使用 O_NONBLOCK 防止 read 卡死，配合 select 实现超时
    fd = open(serial_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "open %s failed: %s\n", serial_name, strerror(errno));
        return 0;
    }

    tcgetattr(fd, &old_tios);
    tcsetattr(fd, TCSANOW, &settings);
    tcflush(fd, TCIOFLUSH);

    // 尝试开启内核 RS485 硬件自动收发切换 (如果 RK3506B 对应引脚支持)
    struct serial_rs485 rs485conf;
    memset(&rs485conf, 0, sizeof(rs485conf));
    rs485conf.flags |= SER_RS485_ENABLED;
    if (ioctl(fd, TIOCSRS485, &rs485conf) < 0) {
        // 如果失败可能使用的是 USB转485，或者需要手动控制 RTS，这里忽略并继续
        // printf("Note: RS485 hardware auto-direction not supported/configured.\n");
    }

    serial_fd = fd;
    return 1;
}

void serial_close(void)
{
    if (serial_fd < 0) return;
    tcsetattr(serial_fd, TCSANOW, &old_tios);
    close(serial_fd);
    serial_fd = -1;
}

/* 发送一帧数据 */
int serial_write(uint8_t *buf, size_t len)
{
    if (serial_fd < 0) return -1;
    return write(serial_fd, buf, len);
}

/* 带超时读取数据 */
int serial_read_timeout(uint8_t *buf, size_t max_len, int timeout_ms)
{
    fd_set rfds;
    struct timeval tv;
    int ret;

    if (serial_fd < 0) return -1;

    FD_ZERO(&rfds);
    FD_SET(serial_fd, &rfds);
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    ret = select(serial_fd + 1, &rfds, NULL, NULL, &tv);
    if (ret > 0) {
        return read(serial_fd, buf, max_len);
    }
    return ret; // 0: 超时, <0: 错误
}

int main(int argc, char *argv[])
{
    uint8_t tx_buf[8] = {0x50, 0x03, 0x00, 0x34, 0x00, 0x03, 0x00, 0x00}; // 模板指令
    uint8_t rx_buf[64];
    int baud_rate;
    int n;

    if (argc < 2) {
        fprintf(stderr, "Usage: %s <serial_dev> [baud_rate]\n", argv[0]);
        fprintf(stderr, "  e.g. %s /dev/ttyS3 9600\n", argv[0]);
        return -1;
    }

    baud_rate = (argc == 3) ? atoi(argv[2]) : 9600;

    // 配置信号退出
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    if (serial_open(argv[1], baud_rate) == 0) {
        return -1;
    }

    // 计算发送帧的 CRC (50 03 00 34 00 03)
    uint16_t crc = modbus_crc16(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;        // CRC 低字节
    tx_buf[7] = (crc >> 8) & 0xFF; // CRC 高字节

    printf("RS485 Modbus Started. Device: %s, Baud: %d, 8N1\n", argv[1], baud_rate);
    printf("Query Frame: ");
    for (int i = 0; i < 8; i++) printf("%02X ", tx_buf[i]);
    printf("\n\n");

    while (g_running) {
        // 1. 发送读取指令
        serial_write(tx_buf, 8);

        // 2. 等待接收响应 (超时设 500ms)
        n = serial_read_timeout(rx_buf, sizeof(rx_buf), 500);

        if (n > 0) {
            // 检查返回数据长度是否足够及站号功能码是否匹配
            if (n >= 11 && rx_buf[0] == 0x50 && rx_buf[1] == 0x03) {
                // 3. 校验 CRC
                uint16_t rx_crc = (rx_buf[n-1] << 8) | rx_buf[n-2];
                uint16_t calc_crc = modbus_crc16(rx_buf, n - 2);

                if (rx_crc == calc_crc) {
                    // 4. 解析加速度数据 (强转为 short 保证负数处理正确)
                    short ax = (short)((rx_buf[3] << 8) | rx_buf[4]);
                    short ay = (short)((rx_buf[5] << 8) | rx_buf[6]);
                    short az = (short)((rx_buf[7] << 8) | rx_buf[8]);

                    // 5. 根据公式转换: 加速度 = 原始值 / 32768 * 16g
                    // 取 g = 9.8 m/s2
                    float ax_g = (float)ax / 32768.0f * 16.0f;
                    float ay_g = (float)ay / 32768.0f * 16.0f;
                    float az_g = (float)az / 32768.0f * 16.0f;

                    printf("RAW: %02X %02X %02X %02X %02X %02X | ", 
                           rx_buf[3], rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7], rx_buf[8]);
                    printf("X: %7.3fg (%6.2f m/s2)  "
                           "Y: %7.3fg (%6.2f m/s2)  "
                           "Z: %7.3fg (%6.2f m/s2)\n",
                           ax_g, ax_g * 9.8f,
                           ay_g, ay_g * 9.8f,
                           az_g, az_g * 9.8f);
                } else {
                    printf("CRC Check Failed! Expected: %04X, Calc: %04X\n", rx_crc, calc_crc);
                }
            } else {
                printf("Invalid Response Format. Len=%d\n", n);
            }
        } else if (n == 0) {
            printf("Timeout: No response from sensor.\n");
        } else {
            if (errno != EINTR) printf("Read error: %s\n", strerror(errno));
        }

        // 采样间隔 1 秒
        sleep(1);
    }

    serial_close();
    printf("Closed.\n");
    return 0;
}
