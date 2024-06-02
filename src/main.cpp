#include "climenu.hpp"
#include "serial.hpp"

#include <string.h>

#include <csignal>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#define MODULE_NAME "Lidar 360 scanner"
#define SAMPLESPERSCAN 360
#define ANGLESTOCHK {0, 45, 90, 135, 180, 225, 270, 315};

#define SCANSTARTFLAG 0xA5
#define SCANGETINFOCMD 0x50
#define SCANGETSTATCMD 0x52
#define SCANGETSRATECMD 0x59
#define SCANSTARTSCAN 0x20
#define SCANSTOPSCAN 0x25

void readinfo(std::shared_ptr<serial> serialIf)
{
    serialIf->write({SCANSTARTFLAG, SCANGETINFOCMD});
    std::vector<uint8_t> resp;
    serialIf->read(resp, 27);

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

void readstatus(std::shared_ptr<serial> serialIf)
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

    serialIf->write({SCANSTARTFLAG, SCANGETSTATCMD});
    std::vector<uint8_t> resp;
    serialIf->read(resp, 10);

    uint8_t status = resp.at(7);
    if (STLAST >= status)
    {
        const char* const info = statesinfo[status];
        printf("Status is: %s\n", info);
    }
    else
    {
        printf("Cannot recognize status: %u\n", status);
    }
}

void readsamplerate(std::shared_ptr<serial> serialIf)
{
    serialIf->write({SCANSTARTFLAG, SCANGETSRATECMD});
    std::vector<uint8_t> resp;
    serialIf->read(resp, 11);

    printf("Single std scan time: %u ms\n", (resp[8] << 8) | resp[7]);
    printf("Single express scan time: %u ms\n", (resp[10] << 8) | resp[9]);
}

void startscanning(std::shared_ptr<serial> serialIf)
{
    serialIf->write({SCANSTARTFLAG, SCANSTARTSCAN});
    std::vector<uint8_t> resp;
    serialIf->read(resp, 7);
    // serialIf->flushBuffer();
}

void stopscanning(std::shared_ptr<serial> serialIf)
{
    serialIf->write({SCANSTARTFLAG, SCANSTOPSCAN});
    std::vector<uint8_t> resp;
    while (serialIf->read(resp, 10))
        ;
    // serialIf->flushBuffer();
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
    for (uint32_t i = 0; i < angarrsize; i++)
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
            {
                qacolor = "\e[1;31m";
            }
            else if (sample[2] <= 60)
            {
                qacolor = "\e[1;33m";
            }
            else
            {
                // green color here
            }

            printf("[%d] angle: \e[4m%3u\260\e[0m,"
                   " dist: \e[4m%5.1fcm\e[0m,"
                   " confid: %s%3u%\e[0m\n",
                   i + 1, sample[0], (double)sample[1] / 10, qacolor,
                   sample[2]);
        }
    }

    printf("\e[?25h");
}

void readscanning(std::shared_ptr<serial> serialIf)
{
    static int samplesarr[SAMPLESPERSCAN][3] = {0};

    system("clear");
    printf("\033[1;1H");
    printf("Standard 360 scan started on\n"
           "%s> Press enter to stop\n",
           gettimestr());

    startscanning(serialIf);
    for (int smpidx = 0; smpidx < SAMPLESPERSCAN; smpidx++)
    {
        std::vector<uint8_t> sample;
        serialIf->read(sample, 5, 1000);
        uint32_t quality = sample[0] >> 2, newscan = sample[0] & 0x01,
                 angle = ((sample[2] << 7) | (sample[1] >> 1)) / 64,
                 distance = ((sample[4] << 8) | sample[3]) / 4;

        if (newscan)
        {
            displaysamples(samplesarr, smpidx);
            memset(samplesarr, 0, sizeof(samplesarr));
            smpidx = 0;
        }

        samplesarr[smpidx][0] = angle;
        samplesarr[smpidx][1] = distance;
        samplesarr[smpidx][2] = 100 * quality / 15;

        if (Menu::isenterpressed())
        {
            break;
        }
    }
    stopscanning(serialIf);
}

void exitprogram([[maybe_unused]] std::shared_ptr<serial> serialIf)
{
    printf("Cleaning and closing\n");
    exit(0);
}

void signalHandler(int signal)
{
    if (signal == SIGINT)
    {
        throw std::runtime_error("Safe app termiantion");
        ;
    }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv[])
{
    std::signal(SIGINT, signalHandler);

    try
    {
        std::shared_ptr<serial> serialIf =
            std::make_shared<usb>("/dev/ttyUSB0", B115200);

        Menu menu{
            "Lidar 360 scanner",
            {{"to get info", std::bind(readinfo, serialIf)},
             {"to get status", std::bind(readstatus, serialIf)},
             {"to get sampling time", std::bind(readsamplerate, serialIf)},
             {"to start scanning", std::bind(readscanning, serialIf)},
             {"exit", std::bind(exitprogram, serialIf)}}};

        menu.run();
    }
    catch (const std::exception& ex)
    {
        std::cerr << ex.what() << "\n";
        stopscanning(std::make_shared<usb>("/dev/ttyUSB0", B115200));
    }
    catch (...)
    {
        std::cerr << "Unknown exception occured, aborting!\n";
    }
    return 0;
}
