#include "scan.hpp"

#include "helpers.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <future>
#include <stdexcept>

constexpr auto MAXANGLEPERSCAN = 360;
constexpr auto SCANSTARTFLAG = 0xA5;
constexpr auto SCANSTARTSCAN = 0x20;
constexpr auto SCANSTOPSCAN = 0x25;

Scan::Scan(std::shared_ptr<serial> serialIf, scan_t type, scansub_t subtype) :
    type{type}, subtype{subtype}, serialIf{serialIf}
{}

Scan::~Scan()
{
    stop();
}

void Scan::stop()
{
    if (isrunning())
    {
        running = false;
        scanning->wait();
    }
}

void Scan::addangle(int32_t angle, const NotifyFunc& func)
{
    observer.event(angle, func);
}

void Scan::delangle([[maybe_unused]] int32_t angle)
{}

scan_t Scan::gettype() const
{
    return type;
}

scansub_t Scan::getsubtype() const
{
    return subtype;
}

std::string Scan::gettypename() const
{
    static const std::unordered_map<scan_t, std::string> typetoname = {
        {scan_t::normal, "normal"}, {scan_t::express, "express"}};
    if (typetoname.contains(type))
    {
        return typetoname.at(type);
    }
    throw std::runtime_error("Cannot find scan type to name mappping");
}

std::string Scan::getsubtypename() const
{
    static const std::unordered_map<scansub_t, std::string> subtypetoname = {
        {scansub_t::none, ""},
        {scansub_t::legacy, "legacy"},
        {scansub_t::dense, "dense"}};
    if (subtypetoname.contains(subtype))
    {
        return subtypetoname.at(subtype);
    }
    throw std::runtime_error("Cannot find scan subtype to name mappping");
}

bool Scan::isrunning() const
{
    return running;
}

void Scan::releasescan()
{
    serialIf->write({SCANSTARTFLAG, SCANSTOPSCAN});
    std::vector<uint8_t> resp;
    while (serialIf->read(resp, 250))
        ;
}

Measurement ScanNormal::getdata(bool firstread = false)
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
                        "Received packet size is incorrect");
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

void ScanNormal::run()
{
    if (!isrunning())
    {
        running = true;
        scanning = std::make_shared<std::future<void>>(
            std::async(std::launch::async, [self = shared_from_this()] {
                self->requestscan();
                auto [isvalid, data] = self->getdata(true);
                while (self->isrunning())
                {
                    if (isvalid)
                    {
                        self->observer.update(data);
                    }
                    std::tie(isvalid, data) = self->getdata();
                }
                self->releasescan();
            }));
    }
}

void ScanNormal::requestscan()
{
    serialIf->write({SCANSTARTFLAG, SCANSTARTSCAN});
    std::vector<uint8_t> resp;
    serialIf->read(resp, 7);
}

void ScanExpress::requestscan()
{
    serialIf->write(
        {SCANSTARTFLAG, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22});
    std::vector<uint8_t> resp;
    serialIf->read(resp, 7);
}

std::pair<double, std::vector<uint8_t>>
    ScanExpress::getbasedata(bool firstread = false)
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
                    throw std::runtime_error("First packet without start flag");
                }

                size_t remainingsize = packetsize - recvsize;
                recvsize += serialIf->read(raw, remainingsize);
                if (recvsize != packetsize)
                {
                    throw std::runtime_error(
                        "Received packet size is incorrect");
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
    ScanExpressLegacy::getcabindata(std::vector<uint8_t>&& cabin,
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

void ScanExpressLegacy::run()
{
    if (!isrunning())
    {
        running = true;
        scanning = std::make_shared<std::future<void>>(
            std::async(std::launch::async, [self = shared_from_this()] {
                self->requestscan();
                [[maybe_unused]] bool newscan{false};
                auto [startangleprev, cabindataprev] = self->getbasedata(true);
                while (self->isrunning())
                {
                    auto [startanglecurr, cabindatacurr] = self->getbasedata();
                    auto angledelta = startanglecurr - startangleprev;
                    if (angledelta < 0)
                    {
                        angledelta += MAXANGLEPERSCAN;
                        newscan = true;
                    }

                    static constexpr uint8_t bytespercabin{5};
                    for (auto it = cabindataprev.cbegin();
                         it != cabindataprev.cend();
                         std::advance(it, bytespercabin))
                    {
                        auto cabinnum = static_cast<uint8_t>(
                            std::distance(cabindataprev.cbegin(), it) /
                            bytespercabin);
                        auto measurements = self->getcabindata(
                            {it, it + bytespercabin}, startangleprev,
                            angledelta, cabinnum);

                        std::ranges::for_each(
                            measurements, [self](Measurement& measurement) {
                                auto [isvalid, data] = measurement;
                                if (isvalid)
                                    self->observer.update(data);
                            });
                    }
                    startangleprev = startanglecurr;
                    cabindataprev = cabindatacurr;
                }
                self->releasescan();
            }));
    }
}

Measurement ScanExpressDense::getcabindata(std::vector<uint8_t>&& cabin,
                                           double startangle, double angledelta,
                                           uint8_t cabinnum)
{
    auto calcangle = [startangle, angledelta](uint8_t measurement) {
        double angle = startangle + (angledelta * measurement / 40.);
        return static_cast<uint32_t>(std::lround(
            angle < MAXANGLEPERSCAN ? angle : angle - MAXANGLEPERSCAN));
    };

    int32_t angle{};
    uint32_t distance = (cabin[0] & 0xFF) | (cabin[1] << 8);

    bool isvalid = distance != 0;
    if (isvalid)
    {
        angle = calcangle(cabinnum + 1);
    }

    return {isvalid, {angle, distance / 10.}};
}

void ScanExpressDense::run()
{
    if (!isrunning())
    {
        running = true;
        scanning = std::make_shared<std::future<void>>(
            std::async(std::launch::async, [self = shared_from_this()] {
                self->requestscan();
                [[maybe_unused]] bool newscan{false};
                auto [startangleprev, cabindataprev] = self->getbasedata(true);

                while (self->isrunning())
                {
                    auto [startanglecurr, cabindatacurr] =
                        self->getbasedata(false);
                    auto angledelta = startanglecurr - startangleprev;
                    if (angledelta <= 0)
                    {
                        angledelta += MAXANGLEPERSCAN;
                        newscan = true;
                    }

                    static constexpr uint8_t bytespercabin{2};
                    for (auto it = cabindataprev.cbegin();
                         it != cabindataprev.cend();
                         std::advance(it, bytespercabin))
                    {
                        auto cabinnum = static_cast<uint8_t>(
                            std::distance(cabindataprev.cbegin(), it) /
                            bytespercabin);
                        auto [isvalid, data] = self->getcabindata(
                            {it, it + bytespercabin}, startangleprev,
                            angledelta, cabinnum);
                        if (isvalid)
                        {
                            self->observer.update(data);
                        }
                    }
                    startangleprev = startanglecurr;
                    cabindataprev = cabindatacurr;
                }
                self->releasescan();
            }));
    }
}
