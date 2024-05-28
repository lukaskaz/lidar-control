#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

//#define UART_DEV	"/dev/ttyS0"
#define UART_DEV "/dev/ttyUSB0"
#define OPTBAUDRATE B115200
#define MODULE_NAME "Lidar 360 scanner"
#define SAMPLESPERSCAN 360
#define ANGLESTOCHK {0, 45, 90, 135, 180, 225, 270, 315};

#define SCANSTARTFLAG 0xA5
#define SCANGETINFOCMD 0x50
#define SCANGETSTATCMD 0x52
#define SCANGETSRATECMD 0x59
#define SCANSTARTSCAN 0x20
#define SCANSTOPSCAN 0x25

typedef enum
{
    OP_GET_NONE = 0,
    OP_GET_FIRST,
    OP_GET_INFO = OP_GET_FIRST,
    OP_GET_STATUS,
    OP_GET_SAMPLERATE,
    OP_GET_SCAN,
    OP_GET_EXIT,
    OP_GET_LAST = OP_GET_EXIT
} op_t;

typedef void (*ophdlr_ptr)(int);

int configureSerial(int fd);

op_t getusersel(void)
{
    op_t selection = OP_GET_NONE;

    while (1)
    {
        int addchars = 0;

        system("clear");
        printf("== " MODULE_NAME " menu ==\n");
        printf("%u : to get info\n", OP_GET_INFO);
        printf("%u : to get status\n", OP_GET_STATUS);
        printf("%u : to get sampling time\n", OP_GET_SAMPLERATE);
        printf("%u : to start scanning\n", OP_GET_SCAN);
        printf("%u : to exit\n", OP_GET_EXIT);
        printf("-> ");

        selection = (op_t)getchar() - '0';
        while ('\n' != getchar())
            addchars++;
        if (OP_GET_FIRST > selection || OP_GET_LAST < selection || addchars > 0)
        {
            printf("Unsupported selection, press"
                   " enter and try again\n");
            while ('\n' != getchar())
                ;
        }
        else
            break;
    }

    return selection;
}

int isenterpressed(void)
{
    struct pollfd pollInfo = {
        .fd = fileno(stdin), .events = POLLIN, .revents = 0};
    int ret = poll(&pollInfo, 1, 0);
    if (0 < ret && 0 != (pollInfo.revents & POLLIN))
    {
        int inchar = getchar();
        if ('\n' == inchar)
            return 1;
        else
            while ('\n' != getchar())
                ;
    }

    return 0;
}

int sendcmdtoscanner(int fd, int cmd)
{
    unsigned char data[] = {SCANSTARTFLAG, cmd};
    int ret = write(fd, data, sizeof(data));

    return (0 < ret) ? 0 : (-1);
}

int recvdatafromscanner(int fd, char data[], int size)
{
    int bytes = 0;

    if (NULL != data && 0 < size)
    {
        struct pollfd pollInfo = {.fd = fd, .events = POLLIN, .revents = 0};
        int timeoutms = 200, ret = 0;

        for (int i = 0; i < size; i++)
        {
            ret = poll(&pollInfo, 1, timeoutms);
            if (0 < ret && 0 != (pollInfo.revents & POLLIN))
            {
                bytes += read(fd, &data[i], 1);
            }
            else if (0 == ret)
            {
                // timeout occured, abort
                break;
            }
            else
            {
                // error occured, abort
                break;
            }
        }
    }

    return (size == bytes) ? 0 : (-1);
}

void readinfo(int fd)
{
    unsigned char resp[27 + 1] = {0};

    sendcmdtoscanner(fd, SCANGETINFOCMD);
    recvdatafromscanner(fd, resp, sizeof(resp) - 1);

    printf("Model: 0x%02X\n", resp[7]);
    printf("Firmware: %u.%u\n", resp[9], resp[8]);
    printf("Hardware: 0x%02X\n", resp[10]);
    printf("Serial number:");
    for (int i = 11, j = 0; i < 27; i++, j++)
        if (j % 4)
            printf("0x%02X ", resp[i]);
        else
            printf("\n0x%02X ", resp[i]);
    printf("\n");
}

void readstatus(int fd)
{
    enum states
    {
        STFIRST = 0,
        STGOOD = STFIRST,
        STWARN,
        STERR,
        STLAST = STERR
    };
    static const char* statesinfo[] = {[STGOOD] = "\e[1;32mOk\e[0m",
                                       [STWARN] = "\e[1;33mWarning\e[0m",
                                       [STERR] = "\e[1;31mERROR\e[0m"};
    unsigned char resp[10 + 1] = {0}, *status = &resp[7];

    sendcmdtoscanner(fd, SCANGETSTATCMD);
    recvdatafromscanner(fd, resp, sizeof(resp) - 1);

    if (STFIRST <= *status && STLAST >= *status)
    {
        const char* const info = statesinfo[*status];
        printf("Status is: %s\n", info);
    }
    else
    {
        printf("Cannot recognize status: %u\n", *status);
    }
}

void readsamplerate(int fd)
{
    unsigned char resp[11 + 1] = {0};

    sendcmdtoscanner(fd, SCANGETSRATECMD);
    recvdatafromscanner(fd, resp, sizeof(resp) - 1);

    printf("Single std scan time: %u ms\n", (resp[8] << 8) | resp[7]);
    printf("Single express scan time: %u ms\n", (resp[10] << 8) | resp[9]);
}

void stopscanning(int fd)
{
    sendcmdtoscanner(fd, SCANSTOPSCAN);
    usleep(2000);
    ioctl(fd, TCFLSH, TCIOFLUSH);
}

const char* gettimestr(void)
{
    time_t rawtime = {0};
    struct tm* timeinfo = {0};

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    return asctime(timeinfo);
}

int comparearr2d(const void* arr1, const void* arr2)
{
    const unsigned int* one = (const unsigned int*)arr1;
    const unsigned int* two = (const unsigned int*)arr2;

    if (one[0] < two[0])
        return -1;
    else if (one[0] > two[0])
        return 1;
    else
        return 0;
}

void displaysamples(int samples[][3], int amount)
{
    static const int angtoshow[] = ANGLESTOCHK;
    static const size_t angarrsize = sizeof(angtoshow) / sizeof(*angtoshow);

    // align first line and hide cursor
    printf("\033[5;1H\e[?25l");
    qsort(samples, amount, sizeof(*samples), comparearr2d);
    for (int i = 0; i < angarrsize; i++)
    {
        int* sample = NULL;

        for (int j = 0; j < amount; j++)
        {
            const int angle = samples[j][0], quality = samples[j][2];

            if (angle >= angtoshow[i] && 0 != quality)
            {
                sample = samples[j];
                break;
            }
        }

        if (NULL != sample)
        {
            const char* qacolor = "\e[1;32m";

            if (sample[2] <= 30)
                qacolor = "\e[1;31m";
            else if (sample[2] <= 60)
                qacolor = "\e[1;33m";
            else
                ; // green color here

            printf("[%d] angle: \e[4m%3u\260\e[0m,"
                   " dist: \e[4m%5.1fcm\e[0m,"
                   " confid: %s%3u%\e[0m\n",
                   i + 1, sample[0], (double)sample[1] / 10, qacolor,
                   sample[2]);
        }
    }

    printf("\e[?25h");
}

void readscanning(int fd)
{
    static int samplesarr[SAMPLESPERSCAN][3] = {0};
    unsigned char resp[7 + 1] = {0}, sample[5 + 1] = {0};

    // stopscanning(fd);

    system("clear");
    printf("\033[1;1H");
    printf("Standard 360 scan started on\n"
           "%s> Press enter to stop\n",
           gettimestr());

    sendcmdtoscanner(fd, SCANSTARTSCAN);
    recvdatafromscanner(fd, resp, sizeof(resp) - 1);

    for (int smpidx = 0; smpidx < SAMPLESPERSCAN; smpidx++)
    {
        recvdatafromscanner(fd, sample, sizeof(sample) - 1);
        int quality = sample[0] >> 2, newscan = sample[0] & 0x01,
            angle = ((sample[2] << 7) | (sample[1] >> 1)) / 64,
            distance = ((sample[4] << 8) | sample[3]) / 4;

        if (0 != newscan)
        {
            displaysamples(samplesarr, smpidx);
            memset(samplesarr, 0, sizeof(samplesarr));
            smpidx = 0;
        }

        samplesarr[smpidx][0] = angle;
        samplesarr[smpidx][1] = distance;
        samplesarr[smpidx][2] = 100 * quality / 15;

        if (1 == isenterpressed())
            break;
    }

    stopscanning(fd);
}

void exitprogram(int fd)
{
    printf("Cleaning and closing\n");
    close(fd);
    exit(0);
}

static ophdlr_ptr ophdlr[] = {
    [OP_GET_NONE] = NULL,         [OP_GET_INFO] = readinfo,
    [OP_GET_STATUS] = readstatus, [OP_GET_SAMPLERATE] = readsamplerate,
    [OP_GET_SCAN] = readscanning, [OP_GET_EXIT] = exitprogram};

int disableHwFlow(int fd)
{
    return ioctl(fd, TIOCMBIC, &(int){TIOCM_DTR | TIOCM_RTS});
}

int configureSerial(int fd)
{
    struct termios options = {0};
    tcgetattr(fd, &options);
    cfsetispeed(&options, OPTBAUDRATE);
    cfsetospeed(&options, OPTBAUDRATE);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIFLUSH);
    fcntl(fd, F_SETFL, FNDELAY);
    return 0;
}

int initserialif(int* fd)
{
    if (NULL != fd)
    {
        *fd = open(UART_DEV, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (0 <= *fd)
        {
            disableHwFlow(*fd);
            configureSerial(*fd);
            return 0;
        }
    }

    return (-1);
}

int main(int argc, char* argv[])
{
    int fd = (-1), ret = initserialif(&fd);
    if (0 == ret)
    {
        while (1)
        {
            op_t usersel = getusersel();
            if (NULL != ophdlr[usersel])
                ophdlr[usersel](fd);
            printf("\nPress enter to return to menu\n");
            getchar();
        }
    }
    else
    {
        printf("Cannot initialize serial interface\n");
    }

    return 0;
}
