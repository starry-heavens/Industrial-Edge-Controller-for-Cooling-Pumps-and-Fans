#include <stdio.h>      /* Standard input/output */
#include <fcntl.h>      /* File control definitions */
#include <string.h>
#include <stdlib.h>
#include <termio.h>     /* POSIX terminal control definitions */
#include <sys/time.h>   /* Time structures for select() */
#include <unistd.h>     /* POSIX Symbolic Constants */
#include <assert.h>
#include <errno.h>      /* Error definitions */
#include <sys/mman.h>


#define  CMSPAR 010000000000  

#define SERIAL_FLUSH_TX		1
#define SERIAL_FLUSH_RX		2

#define SERIAL_PARITY_NO        0
#define SERIAL_PARITY_ODD       1
#define SERIAL_PARITY_EVENT     2
#define SERIAL_PARITY_MARK      3
#define SERIAL_PARITY_SPACE     4

#define SERIAL_STOPBIT_ONE      1
#define SERIAL_STOPBIT_TWO      2

static int serial_fd  = -1;

struct termios old_tios;

static void serial_print_buf(unsigned char *buf,int len)
{
    int i;
    for(i=0;i<len;i++)
        printf("%02X ",buf[i]);
    printf("\n");
}

char *get_serial_dev_name(char *serial_name)
{
    if (strcasecmp(serial_name, "COM1") == 0)
        return "/dev/ttyS1";
    else if (strcasecmp(serial_name, "COM2") == 0)
        return "/dev/ttyS2";
    else if (strcasecmp(serial_name, "COM3") == 0)
        return "/dev/ttyS3";
    else if (strcasecmp(serial_name, "COM4") == 0)
        return "/dev/ttyS4";
    else if (strcasecmp(serial_name, "COM5") == 0)
        return "/dev/ttyS5";
    else if (strcasecmp(serial_name, "COM6") == 0)
        return "/dev/ttyS6";
    else if (strcasecmp(serial_name, "COM7") == 0)
        return "/dev/ttyS7";
    else if (strcasecmp(serial_name, "COM9") == 0)
        return "/dev/ttyS9";
    else
        //return NULL;
        return serial_name;
}

static int termios_init(struct termios *tios,int baud,int parity,int data_bits,int stop_bits)
{
    speed_t baud_rate;

    if (tios == NULL)
        return -1;

    tios->c_line  = 0;

    tios->c_cc[VMIN ] = 0;
    tios->c_cc[VTIME] = 0;

    /* configure the input modes... */
    tios->c_iflag =  IGNBRK | IGNPAR | INPCK;


    /* configure the output modes... */
    tios->c_oflag = 0;     /* enable implementation-defined output processing */
    /* configure the control modes... */
    tios->c_cflag = CREAD | CLOCAL; 

    if (data_bits == 5)
        tios->c_cflag |= CS5;
    else if (data_bits == 6)
        tios->c_cflag |= CS6;
    else if (data_bits == 7)
        tios->c_cflag |= CS7;
    else if (data_bits == 8)
        tios->c_cflag |= CS8;
    else
        return -1;

    if (stop_bits == 1)
        tios->c_cflag &=~ CSTOPB;
    else if (stop_bits == 2)
        tios->c_cflag |= CSTOPB;
    else
        return -1;

    if(parity == 0)
    { /* none */
        tios->c_cflag &=~ PARENB;
        tios->c_cflag &=~ PARODD;
    }
    else if(parity == 2)
    { /* even */
        tios->c_cflag |= PARENB;
        tios->c_cflag &=~ PARODD;
    }
    else if(parity == 1)
    { /* odd */
        tios->c_cflag |= PARENB;
        tios->c_cflag |= PARODD;
    }
    else if (parity == 3)/* 2008.12.17 add for support mark and space parity */
    {
        /* mark */
        tios->c_cflag |= PARENB;
        tios->c_cflag |= CMSPAR;
        tios->c_cflag |= PARODD;
    }
    else if (parity == 4)
    {
        /* space */
        tios->c_cflag |= PARENB;
        tios->c_cflag |= CMSPAR;
    }
    else
        return -1;

    /* configure the local modes... */
    tios->c_lflag = 0;    /* enable implementation-defined input processing   */

    /* Set the baud rate */
    switch(baud)
    {
        case 110:
            baud_rate = B110;
            break;
        case 300:
            baud_rate = B300;
            break;
        case 600:
            baud_rate = B600;
            break;
        case 1200:
            baud_rate = B1200;
            break;
        case 2400:
            baud_rate = B2400;
            break;
        case 4800:
            baud_rate = B4800;
            break;
        case 9600:
            baud_rate = B9600;
            break;
        case 19200:
            baud_rate = B19200;
            break;
        case 38400:
            baud_rate = B38400;
            break;
        case 57600:
            baud_rate = B57600;
            break;
        case 115200:
            baud_rate = B115200;
            break;
        case 230400:
            baud_rate = B230400;
            break;
        case 460800:
            baud_rate = B460800;
            break;
        case 576000:
            baud_rate = B576000;
            break;
        case 921600:
            baud_rate = B921600;
            break;

        default:
            return -1;
    } 

    if ((cfsetispeed(tios, baud_rate) < 0) ||(cfsetospeed(tios, baud_rate) < 0))
        return -1;
    return 0;
}

//return serial fd, error return -1
int serial_open(char *serial_name, int baud,int parity,int data_bits,int stop_bits,int timeout)
{
    struct termios settings;
    int fd;
    char *serial_dev_name;

    if (serial_fd >= 0)
        return 0;
    if (termios_init(&settings, baud,parity,data_bits,stop_bits) < 0)
        return 0;


    serial_dev_name = get_serial_dev_name(serial_name);
    if (serial_dev_name == NULL)
    {
        fprintf(stderr, "invalid serial name:%s\n", serial_name);
        return 0;
    }

    if((fd = open(serial_dev_name, O_RDWR | O_NOCTTY | O_NDELAY))< 0)
        return 0;

    if(tcgetattr(fd, &old_tios) < 0)
    {
        close(fd);
        return 0;
    }

    if(tcsetattr(fd, TCSANOW, &settings) < 0)
    {
        close(fd);
        return 0;
    }

    serial_fd = fd;
    return 1;
}

int serial_close()
{
    if (serial_fd < 0)
        return 0;
    tcsetattr(serial_fd, TCSANOW, &(old_tios));
    close(serial_fd);
    serial_fd = -1;

    return 1;
}

int serial_flush(int flag)
{
    if (serial_fd < 0)
        return 0;

    if (flag == SERIAL_FLUSH_TX)
        tcflush(serial_fd,TCOFLUSH);
    else if (flag == SERIAL_FLUSH_RX)
        tcflush(serial_fd,TCIFLUSH);
    else if (flag == (SERIAL_FLUSH_RX|SERIAL_FLUSH_TX) )
        tcflush(serial_fd,TCIOFLUSH);
    return 1;
}

int serial_write(char *buf,size_t size)
{
    int writesize = 0;

    writesize = write(serial_fd,buf,size);
    //printf("serial write: length=%d, packet: ",writesize);
    //serial_print_buf(buf,writesize);
    return writesize;
}

int serial_read(char *buf,size_t size)
{
    int readsize = 0;

    if (size<=0)
        return 0;

    readsize = read(serial_fd,buf,size);
    printf("serial read: length=%d, packet: ",readsize);
    serial_print_buf(buf, readsize);
    return readsize;
}


//return 1: poll ok, 0: timeout, -1: error
int serial_poll(int timeout)
{
    int fd = serial_fd;

    fd_set rfds;
    struct timeval tv;
    int sel_res;

    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    tv.tv_sec = timeout/1000;
    tv.tv_usec = (timeout % 1000)*1000;
    return select(fd + 1, &rfds, NULL,NULL,&tv);
}

int main(int argc, char *argv[])
{
    int fd;
    char send_buffer[100];
    char recv_buffer[100];
    int i, ret;
    int baud_rate;

    if (argc != 3)  // 检查参数数量
    {
        fprintf(stderr, "usage: %s com_port baud_rate\n", argv[0]);
        return -1;
    }

    // 解析命令行输入的波特率
    baud_rate = atoi(argv[2]);
    if (baud_rate <= 0)
    {
        fprintf(stderr, "Invalid baud rate: %s\n", argv[2]);
        return -1;
    }

    ret = serial_open(argv[1], baud_rate, SERIAL_PARITY_NO, 8, SERIAL_STOPBIT_ONE, 1000);
    if (ret == 0)
    {
        fprintf(stderr, "open serial failed\n");
        return -1;
    }        

    while(1)
    {

        for(i = 0; i < 20; i++)
        {
            send_buffer[i] = (i % 2 == 0) ? 0x01 : 0x01;  // 交替填充0x01
        }
        serial_write(send_buffer, 20);
        printf("Sent data: ");
        for (i = 0; i < 20; i++)
        {
            printf("%02X ", send_buffer[i]);  // 打印十六进制格式
        }
        printf("\n");

        // read
        while (serial_poll(1000) > 0)
        {
            int bytes_read = serial_read(recv_buffer, sizeof(recv_buffer));
            if (bytes_read > 0)
            {
                //printf("Received %d bytes: ", bytes_read);
                for (i = 0; i < bytes_read; i++)
                {
                    printf("%02X ", recv_buffer[i]);  // 打印十六进制格式
                }
                printf("\n");
            }
        }
    }

    serial_close();    

    return 0;
}
