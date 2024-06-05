#include "climenu.hpp"
#include "serial.hpp"

#include <string.h>

#include <boost/program_options.hpp>

#include <array>
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

using Samples = std::map<uint32_t, std::pair<uint32_t, uint32_t>>;

void displaysamples(const Samples& samples)
{
    std::array angletoshow = ANGLESTOCHK;

    // align first line and hide cursor
    printf("\033[5;1H\e[?25l");
    auto showAngleIt = angletoshow.begin();
    for (const auto& [angle, measure] : samples)
    {
        const auto& [distance, quality] = measure;
        if (angle >= *showAngleIt && quality > 0)
        {
            const char* qacolor = "\e[1;32m";
            if (quality <= 30)
            {
                qacolor = "\e[1;31m"; // red status
            }
            else if (quality <= 60)
            {
                qacolor = "\e[1;33m"; // yellow status
            }
            else
            {
                // default green status
            }

            auto pos = std::distance(angletoshow.begin(), ++showAngleIt);
            printf("[%zd] angle: \e[4m%3u\260\e[0m, dist: \e[4m%5.1fcm\e[0m, "
                   "confid: %s%3u%\e[0m\n",
                   pos, angle, distance / 10.0, qacolor, quality);

            if (pos == angletoshow.size())
            {
                printf("\nCollected samples amount: [%zu]\n", samples.size());
                break;
            }
        }
    }
    printf("\e[?25h");
}

void readscanning(std::shared_ptr<serial> serialIf)
{
    system("clear");
    printf("\033[1;1HNormal 360 scan started on\n%s> Press enter to stop\n",
           gettimestr());

    Samples samples;
    startscanning(serialIf);
    while (!Menu::isenterpressed())
    {
        std::vector<uint8_t> raw;
        serialIf->read(raw, 5, 1000);

        bool newscan = raw[0] & 0x01;
        if (!newscan)
        {
            const uint32_t quality = 100 * (raw[0] >> 2) / 15,
                           angle = ((raw[2] << 7) | (raw[1] >> 1)) / 64,
                           distance = ((raw[4] << 8) | raw[3]) / 4;
            samples.try_emplace(angle, distance, quality);
        }
        else
        {
            displaysamples(std::move(samples));
            samples.clear();
        }
    }
    stopscanning(serialIf);
}

void exitprogram()
{
    printf("Cleaning and closing\n");
    exit(0);
}

void signalHandler(int signal)
{
    if (signal == SIGINT)
    {
        throw std::runtime_error("Safe app termiantion");
    }
}

int main(int argc, char* argv[])
{
    std::signal(SIGINT, signalHandler);
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()("help,h", "produce help message")(
        "device,d", boost::program_options::value<std::string>(),
        "serial device node")("speed,s",
                              boost::program_options::value<std::string>(),
                              "speed of serial communication");

    boost::program_options::variables_map vm;
    boost::program_options::store(
        boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc;
        return 0;
    }

    const auto& device =
        vm.count("device") ? vm.at("device").as<std::string>() : "/dev/ttyUSB0";
    try
    {
        std::shared_ptr<serial> serialIf =
            std::make_shared<usb>(device, B115200);

        Menu menu{"[Lidar 360 scanner on " + device + "]",
                  {{"get info", std::bind(readinfo, serialIf)},
                   {"get status", std::bind(readstatus, serialIf)},
                   {"get sampling time", std::bind(readsamplerate, serialIf)},
                   {"start normal scanning", std::bind(readscanning, serialIf)},
                   {"start express scanning",
                    []() { std::cout << "[In progress...]\n"; }},
                   {"exit", exitprogram}}};

        menu.run();
    }
    catch (const std::exception& ex)
    {
        std::cerr << ex.what() << "\n";
        stopscanning(std::make_shared<usb>(device, B115200));
    }
    catch (...)
    {
        std::cerr << "Unknown exception occured, aborting!\n";
    }
    return 0;
}
