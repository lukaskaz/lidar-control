#include "climenu.hpp"
#include "serial.hpp"

#include <string.h>

#include <boost/program_options.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <csignal>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#define MODULE_NAME "Lidar 360 scanner"
#define SAMPLESPERSCAN 360
#define ANGLESTOCHK {0U, 45U, 90U, 135U, 180U, 225U, 270U, 315U};

#define SCANSTARTFLAG 0xA5
#define SCANGETINFOCMD 0x50
#define SCANGETSTATCMD 0x52
#define SCANGETSRATECMD 0x59
#define SCANGETCONFCMD 0x84
#define SCANSTARTSCAN 0x20
#define SCANSTOPSCAN 0x25

using NormalSamples = std::map<float, std::pair<int32_t, int32_t>>;
using ExpressSamples = std::map<uint32_t, double>;
using Measurement = std::pair<bool, std::pair<uint32_t, double>>;

uint8_t getchecksum(const std::vector<uint8_t>& data)
{
    uint8_t chsum{};
    std::for_each(data.begin(), data.end(),
                  [&chsum](const uint8_t byte) { chsum ^= byte; });
    return chsum;
}

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

void showserialtraces(std::string_view name, const std::vector<uint8_t>& packet)
{
    uint8_t maxcolumn = 4, column{maxcolumn};
    std::cout << std::dec << "\n> Name: " << name << ", size: " << packet.size()
              << std::hex << "\n";
    for (const auto& byte : packet)
    {
        std::cout << "0x" << (uint32_t)byte << " ";
        if (!--column)
        {
            column = maxcolumn;
            std::cout << "\n";
        }
    }
}

void readpacket(std::shared_ptr<serial> serialIf, std::vector<uint8_t>&& req,
                std::vector<uint8_t>& resp, uint8_t size, bool chsum = false,
                bool debug = false)
{
    if (chsum)
    {
        req.push_back(getchecksum(req));
    }
    serialIf->write(req);
    serialIf->read(resp, size);

    if (debug)
    {
        showserialtraces("Req", req);
        showserialtraces("Resp", resp);
    }
}

void readconfiguration(std::shared_ptr<serial> serialIf)
{
    constexpr uint8_t reqdatasize = 4, respdatasize = reqdatasize,
                      resppacketsize = respdatasize + 7;

    std::vector<uint8_t> resp;
    readpacket(serialIf,
               {SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize, 0x70, 0, 0, 0},
               resp, resppacketsize + 2, true);
    uint32_t modescount = resp[12] << 8 | resp[11];
    std::cout << "Scan modes count: " << modescount << "\n";

    resp.clear();
    readpacket(serialIf,
               {SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize, 0x7C, 0, 0, 0},
               resp, resppacketsize + 2, true);
    uint32_t typicalmode = resp[12] << 8 | resp[11];
    std::cout << "Typical scan mode: " << typicalmode << "\n";

    for (uint8_t mode = 0; mode < modescount; mode++)
    {
        resp.clear();
        readpacket(serialIf,
                   {SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize + 2, 0x7F, 0, 0,
                    0, mode, 0},
                   resp, resppacketsize + 200, true);
        std::string name(resp.begin() + 11, resp.end());
        std::cout << "Info for mode " << (int)mode << " [" << name << "]";
        if (mode == typicalmode)
        {
            std::cout << " -> TYPICAL";
        }
        std::cout << "\n";

        resp.clear();
        readpacket(serialIf,
                   {SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize + 2, 0x71, 0, 0,
                    0, mode, 0},
                   resp, resppacketsize + 4, true);

        uint32_t uscostpersample =
            resp[14] << 24 | resp[13] << 16 | resp[12] << 8 | resp[11];
        uscostpersample /= 256;
        std::cout << "\t> cost per sample: " << uscostpersample << " us\n";
        std::cout << "\t> max sample rate: " << 1000 * 1000 / uscostpersample
                  << " sps\n";

        resp.clear();
        readpacket(serialIf,
                   {SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize + 2, 0x74, 0, 0,
                    0, mode, 0},
                   resp, resppacketsize + 4, true);

        uint32_t maxdistance =
            resp[14] << 24 | resp[13] << 16 | resp[12] << 8 | resp[11];
        maxdistance /= 256;
        std::cout << "\t> max distance: " << maxdistance << "m\n";

        resp.clear();
        readpacket(serialIf,
                   {SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize + 2, 0x75, 0, 0,
                    0, mode, 0},
                   resp, resppacketsize + 1, true);

        uint32_t answercmdtype = resp[11];
        std::cout << "\t> answer cmd type: " << std::hex << std::showbase
                  << answercmdtype << std::noshowbase << std::dec << "\n";
        std::cout << "\n";
    }
}

void startnormalscanning(std::shared_ptr<serial> serialIf)
{
    serialIf->write({SCANSTARTFLAG, SCANSTARTSCAN});
    std::vector<uint8_t> resp;
    serialIf->read(resp, 7);
    // serialIf->flushBuffer();
}

void startexpressscanning(std::shared_ptr<serial> serialIf)
{
    serialIf->write(
        {SCANSTARTFLAG, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22});
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
    time_t rawtime{0};
    time(&rawtime);
    return asctime(localtime(&rawtime));
}

void displaysamples(const NormalSamples& samples)
{
    std::array angletoshow = ANGLESTOCHK;
    // align first line and hide cursor
    printf("\033[5;1H\e[?25l");
    auto showAngleIt = angletoshow.begin();
    for (const auto& [angle, measure] : samples)
    {
        const auto& [distance, quality] = measure;
        if (angle >= static_cast<float>(*showAngleIt))
        {
            static constexpr auto qacolor = [](uint32_t quality) {
                return quality <= 30 ? "\e[1;31m"                 /* red */
                                     : quality <= 60 ? "\e[1;33m" /* yellow */
                                                     : "\e[1;32m" /* green */;
            };

            auto pos = std::distance(angletoshow.begin(), ++showAngleIt);
            printf("[%zd] angle: \e[4m%3.0f\260\e[0m, dist: \e[4m%5.1fcm\e[0m, "
                   "confid: %s%3u%\e[0m\n",
                   pos, angle, distance / 10.0, qacolor(quality), quality);

            if (pos == angletoshow.size())
            {
                printf("\nCollected samples amount: [%zu]\n", samples.size());
                break;
            }
        }
    }
    printf("\e[?25h");
}

void readnormalscanning(std::shared_ptr<serial> serialIf)
{
    system("clear");
    printf("\033[1;1HNormal 360 scan started on\n%s> Press enter to stop\n",
           gettimestr());

    NormalSamples samples;
    startnormalscanning(serialIf);
    while (!Menu::isenterpressed())
    {
        std::vector<uint8_t> raw;
        serialIf->read(raw, 5, 1000);

        bool newscan = raw[0] & 0x01;
        if (!newscan)
        {
            const float angle =
                static_cast<float>((raw[2] << 7) | (raw[1] >> 1)) / 64.f;
            const uint32_t quality = 100 * (raw[0] >> 2) / 15,
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

void displaysamples(const ExpressSamples& samples)
{
    std::array angletoshow = ANGLESTOCHK;
    // align first line and hide cursor
    printf("\033[5;1H\e[?25l");
    auto showAngleIt = angletoshow.begin();
    for (const auto& [angle, distance] : samples)
    {
        if (angle >= *showAngleIt)
        {
            auto pos = std::distance(angletoshow.begin(), ++showAngleIt);
            printf("[%zd] angle: \e[4m%3d\260\e[0m, dist: \e[4m%5.1fcm\e[0m\n",
                   pos, angle, distance);

            if (pos == angletoshow.size())
            {
                printf("\nCollected samples amount: [%zu]\n", samples.size());
                break;
            }
        }
    }
    printf("\e[?25h");
}

std::pair<double, std::vector<uint8_t>>
    getexpressbasedata(std::shared_ptr<serial> serialIf, bool firstread = false)
{
    static constexpr uint32_t packetsize = 84;
    std::vector<uint8_t> raw;

    auto recvsize = firstread ? serialIf->read(raw, packetsize, 2000)
                              : serialIf->read(raw, packetsize);
    if (recvsize != packetsize)
    {
        throw std::runtime_error(std::string(__func__) +
                                 ": received packet size is incorrect");
    }

    uint8_t sync1 = (raw[0] & 0xF0) >> 4, sync2 = (raw[1] & 0xF0) >> 4;
    if (sync1 != 0xA || sync2 != 0x5)
    {
        throw std::runtime_error(std::string(__func__) +
                                 ": sync bytes not correct");
    }

    bool startflag = raw[3] & 0x80 ? true : false;
    if (firstread && !startflag)
    {
        throw std::runtime_error(std::string(__func__) +
                                 ": first packet has no start flag");
    }

    uint8_t chsumrecv = (uint8_t)((raw[1] & 0x0F) << 4) | (raw[0] & 0x0F);
    uint8_t chsumcalc = getchecksum({raw.begin() + 2, raw.end()});
    if (chsumrecv != chsumcalc)
    {
        throw std::runtime_error(std::string(__func__) +
                                 ": checksum not valid");
    }

    double startangle =
        static_cast<double>(((raw[3] & 0x7F) << 8) | raw[2]) / 64.;
    std::vector<uint8_t> cabindata{raw.begin() + 4, raw.end()};

    return {startangle, std::move(cabindata)};
}

std::array<Measurement, 2> getcabindata(std::vector<uint8_t>&& cabin,
                                        double startangle, double angledelta,
                                        uint8_t cabinnum)
{
    auto calcangle = [startangle, angledelta](uint8_t anglecompq3,
                                              uint8_t measurement) {
        bool isnegative = anglecompq3 & 0x20 ? true : false;
        double anglecomp = (anglecompq3 & 0x1F) / 8.;
        anglecomp *= isnegative ? (-1) : 1;
        double angle =
            startangle + (angledelta * measurement / 32.) - anglecomp;
        return static_cast<uint32_t>(lround(angle < 360 ? angle : angle - 360));
    };

    uint32_t anglefirst{};
    uint32_t distancefirst = ((cabin[0] >> 2) & 0x3F) | (cabin[1] << 6);
    bool isfirstvalid = distancefirst != 0;
    if (isfirstvalid)
    {
        uint8_t anglecompq3 =
                    (cabin[4] & 0x0F) | (uint8_t)((cabin[0] & 0x03) << 4),
                measurement = uint8_t(2 * cabinnum + 0);
        anglefirst = calcangle(anglecompq3, measurement);
    }

    uint32_t anglesecond{};
    uint32_t distancesecond = ((cabin[2] >> 2) & 0x3F) | (cabin[3] << 6);
    bool issecondvalid = distancefirst != 0;
    if (issecondvalid)
    {
        uint8_t anglecompq3 =
                    (uint8_t)(cabin[4] >> 4) | (((cabin[2] & 0x01)) & 0x0F),
                measurement = uint8_t(2 * cabinnum + 1);
        anglesecond = calcangle(anglecompq3, measurement);
    }

    return {{{isfirstvalid, {anglefirst, distancefirst / 10.}},
             {issecondvalid, {anglesecond, distancesecond / 10.}}}};
}

void readexpressscanning(std::shared_ptr<serial> serialIf)
{
    system("clear");
    printf("\033[1;1HExpress 360 scan started on\n%s> Press enter to stop\n",
           gettimestr());

    startexpressscanning(serialIf);

    constexpr uint8_t bytespercabin{5};
    ExpressSamples samples;
    bool newscan{false};
    auto [startangleprev, cabindataprev] = getexpressbasedata(serialIf, true);
    while (!Menu::isenterpressed())
    {
        auto [startanglecurr, cabindatacurr] = getexpressbasedata(serialIf);
        auto angledelta = startanglecurr - startangleprev;
        if (angledelta < 0)
        {
            angledelta += 360;
            newscan = true;
        }

        for (auto it = cabindataprev.cbegin(); it != cabindataprev.cend();
             std::advance(it, bytespercabin))
        {
            uint8_t cabinnum = static_cast<uint8_t>(
                std::distance(cabindataprev.cbegin(), it) / bytespercabin);
            auto measurements = getcabindata(
                {it, it + bytespercabin}, startangleprev, angledelta, cabinnum);

            std::for_each(measurements.begin(), measurements.end(),
                          [&samples](const Measurement& measurement) {
                              const auto& [isvalid, data] = measurement;
                              if (isvalid)
                                  samples.emplace(data);
                          });
        }
        startangleprev = startanglecurr;
        cabindataprev = cabindatacurr;

        if (newscan)
        {
            newscan = false;
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

        Menu menu{
            "[Lidar 360 scanner on " + device + "]",
            {{"get info", std::bind(readinfo, serialIf)},
             {"get status", std::bind(readstatus, serialIf)},
             {"get sampling time", std::bind(readsamplerate, serialIf)},
             {"get configuration", std::bind(readconfiguration, serialIf)},
             {"start normal scanning", std::bind(readnormalscanning, serialIf)},
             {"start express scanning",
              std::bind(readexpressscanning, serialIf)},
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
