#include "scanning.hpp"

#include "climenu.hpp"
#include "helpers.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#define MAXANGLEPERSCAN 360

#define SCANSTARTFLAG 0xA5
#define SCANSTARTSCAN 0x20
#define SCANSTOPSCAN 0x25

void show(const SampleData& data, uint32_t pos)
{
    auto [angle, distance] = data;
    printf("\e[%d;1H[%u] angle(dgr) \e[4m%3u\e[0m, dist(cm): \e[4m%5.1f\e[0m\n",
           5 + pos, pos + 1, angle, distance);
}

void statustest(int32_t& line, auto& observer)
{
    observer.event(0,
                   [line{line}](const SampleData& data) { show(data, line); });
    line++;
    observer.event(45,
                   [line{line}](const SampleData& data) { show(data, line); });
    line++;
    observer.event(90,
                   [line{line}](const SampleData& data) { show(data, line); });
    line++;
    observer.event(135,
                   [line{line}](const SampleData& data) { show(data, line); });
    line++;
    observer.event(180,
                   [line{line}](const SampleData& data) { show(data, line); });
    line++;
    observer.event(225,
                   [line{line}](const SampleData& data) { show(data, line); });
    line++;
    observer.event(270,
                   [line{line}](const SampleData& data) { show(data, line); });
    line++;
    observer.event(315,
                   [line{line}](const SampleData& data) { show(data, line); });
    line++;
    line++;

    observer.event(180, [line{line}](const SampleData& data) {
        const auto& [angle, distance] = data;
        if (distance < 30)
        {
            printf("\e[%d;1H[%03u] CRITICAL: OBSTACLE TOO CLOSE\e[0m\n",
                   5 + line, angle);
        }
        else if (distance < 60)
        {
            printf("\e[%d;1H[%03u] WARNING: OBSTACLE NEARBY    \e[0m\n",
                   5 + line, angle);
        }
        else
        {
            printf("\e[%d;1H[%03u] GOOD: OBSTACLE FAR AWAY     \e[0m\n",
                   5 + line, angle);
        }
    });
    line++;
}

Measurement Normalscan::getdata(bool firstread = false)
{
    static constexpr uint32_t packetsize = 5, poorquality = 10;
    std::vector<uint8_t> raw;

    auto recvsize =
        firstread ? serialIf->read(raw, 1, 2000) : serialIf->read(raw, 1);
    while (true)
    {
        bool flagbit = ((raw[0] >> 1) ^ raw[0]) & 0x01;
        if (flagbit)
        {
            recvsize += serialIf->read(raw, 1);
            bool syncbit = raw[1] & 0x01;
            if (syncbit)
            {
                size_t remainingsize = packetsize - recvsize;
                recvsize += serialIf->read(raw, remainingsize);
                if (recvsize != packetsize)
                {
                    throw std::runtime_error(
                        std::string(__func__) +
                        ": received packet size is incorrect");
                }

                [[maybe_unused]] bool newscan = raw[0] & 0x01;
                uint32_t quality = 100 * (raw[0] >> 2) / 63;

                int32_t angle{};
                double distance{};
                bool isvalid = quality > poorquality;
                if (isvalid)
                {
                    angle = static_cast<int32_t>(
                        lround(((raw[2] << 7) | (raw[1] >> 1)) / 64.));
                    distance = (((raw[4] << 8) | raw[3]) / 4.) / 10.;
                }
                return {isvalid, {angle, distance}};
            }
        }
        raw.clear();
        recvsize = serialIf->read(raw, 1);
    }
}

void Normalscan::run()
{
    system("clear");
    printf("\033[1;1HNormal 360 scan started on\n%s\n"
           "> Press enter to stop\n\033[5;1H\e[?25l",
           gettimestr().c_str());
    requestscan();

    int32_t line{};
    statustest(line, observer);

    auto [isvalid, data] = getdata(true);
    while (!Menu::isenterpressed())
    {
        if (isvalid)
        {
            observer.update(data);
        }
        std::tie(isvalid, data) = getdata();
    }
    printf("\e[%d;1H\n\e[?25h", line + 5);
}

void Normalscan::requestscan()
{
    serialIf->write({SCANSTARTFLAG, SCANSTARTSCAN});
    std::vector<uint8_t> resp;
    serialIf->read(resp, 7);
}

void Normalscan::stopscan()
{
    serialIf->write({SCANSTARTFLAG, SCANSTOPSCAN});
    std::vector<uint8_t> resp;
    while (serialIf->read(resp, 250))
        ;
}

void Expressscan::requestscan()
{
    serialIf->write(
        {SCANSTARTFLAG, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22});
    std::vector<uint8_t> resp;
    serialIf->read(resp, 7);
}

std::pair<double, std::vector<uint8_t>>
    Expressscan::getbasedata(bool firstread = false)
{
    static constexpr uint32_t packetsize = 84;
    std::vector<uint8_t> raw;

    auto recvsize =
        firstread ? serialIf->read(raw, 1, 2000) : serialIf->read(raw, 1);
    while (true)
    {
        uint8_t sync1 = (raw[0] & 0xF0) >> 4;
        if (sync1 == 0x0A)
        {
            recvsize += serialIf->read(raw, 1);
            uint8_t sync2 = (raw[1] & 0xF0) >> 4;
            if (sync2 == 0x05)
            {
                recvsize += serialIf->read(raw, 2);
                bool startflag = raw[3] & 0x80 ? true : false;
                if (firstread && !startflag)
                {
                    throw std::runtime_error(std::string(__func__) +
                                             ": first packet has no start "
                                             "flag, probably hw malfunction");
                }

                size_t remainingsize = packetsize - recvsize;
                recvsize += serialIf->read(raw, remainingsize);
                if (recvsize != packetsize)
                {
                    throw std::runtime_error(
                        std::string(__func__) +
                        ": received packet size is incorrect");
                }

                uint8_t chsumrecv =
                    (uint8_t)((raw[1] & 0x0F) << 4) | (raw[0] & 0x0F);
                uint8_t chsumcalc = getchecksum({raw.begin() + 2, raw.end()});
                if (chsumrecv == chsumcalc)
                {
                    double startangle =
                        static_cast<double>(((raw[3] & 0x7F) << 8) | raw[2]) /
                        64.;
                    std::vector<uint8_t> cabindata{raw.begin() + 4, raw.end()};
                    return {startangle, std::move(cabindata)};
                }
            }
        }
        raw.clear();
        recvsize = serialIf->read(raw, 1);
    }
}

std::array<Measurement, 2>
    Expressscan::getlegacydata(std::vector<uint8_t>&& cabin, double startangle,
                               double angledelta, uint8_t cabinnum)
{
    auto calcangle = [startangle, angledelta](uint8_t anglecompq3,
                                              uint8_t measurement) {
        bool isnegative = anglecompq3 & 0x20 ? true : false;
        double anglecomp = (anglecompq3 & 0x1F) / 8.;
        anglecomp *= isnegative ? (-1) : 1;
        double angle =
            startangle + (angledelta * measurement / 32.) - anglecomp;
        return static_cast<int32_t>(
            lround(angle < MAXANGLEPERSCAN ? angle : angle - MAXANGLEPERSCAN));
    };

    int32_t anglefirst{};
    uint32_t distancefirst = ((cabin[0] >> 2) & 0x3F) | (cabin[1] << 6);

    bool isfirstvalid = distancefirst != 0;
    if (isfirstvalid)
    {
        uint8_t anglecompq3 =
                    (cabin[4] & 0x0F) | (uint8_t)((cabin[0] & 0x03) << 4),
                measurement = uint8_t(2 * cabinnum + 0);
        anglefirst = calcangle(anglecompq3, measurement + 1);
    }

    int32_t anglesecond{};
    uint32_t distancesecond = ((cabin[2] >> 2) & 0x3F) | (cabin[3] << 6);

    bool issecondvalid = distancesecond != 0;
    if (issecondvalid)
    {
        uint8_t anglecompq3 =
                    (uint8_t)(cabin[4] >> 4) | (((cabin[2] & 0x01)) & 0x0F),
                measurement = uint8_t(2 * cabinnum + 1);
        anglesecond = calcangle(anglecompq3, measurement + 1);
    }

    return {{{isfirstvalid, {anglefirst, distancefirst / 10.}},
             {issecondvalid, {anglesecond, distancesecond / 10.}}}};
}

void Expressscan::runlegacy()
{
    system("clear");
    printf("\033[1;1HExpress 360 legacy scan started on\n%s\n"
           "> Press enter to stop\n\033[5;1H\e[?25l",
           gettimestr().c_str());
    requestscan();

    int32_t line{};
    statustest(line, observer);

    constexpr uint8_t bytespercabin{5};
    [[maybe_unused]] bool newscan{false};
    auto [startangleprev, cabindataprev] = getbasedata(true);
    while (!Menu::isenterpressed())
    {
        auto [startanglecurr, cabindatacurr] = getbasedata();
        auto angledelta = startanglecurr - startangleprev;
        if (angledelta < 0)
        {
            angledelta += MAXANGLEPERSCAN;
            newscan = true;
        }

        for (auto it = cabindataprev.cbegin(); it != cabindataprev.cend();
             std::advance(it, bytespercabin))
        {
            auto cabinnum = static_cast<uint8_t>(
                std::distance(cabindataprev.cbegin(), it) / bytespercabin);
            auto measurements = getlegacydata(
                {it, it + bytespercabin}, startangleprev, angledelta, cabinnum);

            std::ranges::for_each(measurements,
                                  [this](Measurement& measurement) {
                                      auto [isvalid, data] = measurement;
                                      if (isvalid)
                                          observer.update(data);
                                  });
        }
        startangleprev = startanglecurr;
        cabindataprev = cabindatacurr;
    }
    printf("\e[%d;1H\n\e[?25h", line + 5);
}

Measurement Expressscan::getdensedata(std::vector<uint8_t>&& cabin,
                                      double startangle, double angledelta,
                                      uint8_t cabinnum)
{
    auto calcangle = [startangle, angledelta](uint8_t measurement) {
        double angle = startangle + (angledelta * measurement / 40.);
        return static_cast<uint32_t>(std::lround(
            angle < MAXANGLEPERSCAN ? angle : angle - MAXANGLEPERSCAN));
    };

    uint32_t angle{};
    uint32_t distance = (cabin[0] & 0xFF) | (cabin[1] << 8);

    bool isvalid = distance != 0;
    if (isvalid)
    {
        angle = calcangle(cabinnum + 1);
    }

    return {isvalid, {angle, distance / 10.}};
}

void Expressscan::rundense()
{
    system("clear");
    printf("\033[1;1HExpress 360 dense scan started on\n%s\n"
           "> Press enter to stop\n\033[5;1H\e[?25l",
           gettimestr().c_str());
    requestscan();

    int32_t line{};
    statustest(line, observer);

    constexpr uint8_t bytespercabin{2};
    [[maybe_unused]] bool newscan{false};
    auto [startangleprev, cabindataprev] = getbasedata(true);
    while (!Menu::isenterpressed())
    {
        auto [startanglecurr, cabindatacurr] = getbasedata(false);
        auto angledelta = startanglecurr - startangleprev;
        if (angledelta <= 0)
        {
            angledelta += MAXANGLEPERSCAN;
            newscan = true;
        }

        for (auto it = cabindataprev.cbegin(); it != cabindataprev.cend();
             std::advance(it, bytespercabin))
        {
            auto cabinnum = static_cast<uint8_t>(
                std::distance(cabindataprev.cbegin(), it) / bytespercabin);
            auto [isvalid, data] = getdensedata(
                {it, it + bytespercabin}, startangleprev, angledelta, cabinnum);
            if (isvalid)
            {
                observer.update(data);
            }
        }
        startangleprev = startanglecurr;
        cabindataprev = cabindatacurr;
    }
    printf("\e[%d;1H\n\e[?25h", line + 5);
}

void Expressscan::stopscan()
{
    serialIf->write({SCANSTARTFLAG, SCANSTOPSCAN});
    std::vector<uint8_t> resp;
    while (serialIf->read(resp, 250))
        ;
}
