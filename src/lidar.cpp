#include "lidar.hpp"

#include "display.hpp"
#include "helpers.hpp"

#include <algorithm>
#include <sstream>
#include <unordered_map>

constexpr auto SCANSTARTFLAG = 0xA5;
constexpr auto SCANGETINFOCMD = 0x50;
constexpr auto SCANGETSTATCMD = 0x52;
constexpr auto SCANGETSRATECMD = 0x59;
constexpr auto SCANGETCONFCMD = 0x84;

void Lidar::menu()
{
    Display(*this, scans)
        .run({model, serialIf->getdevice(), serialIf->getbaud(), ""});
}

bool Lidar::setup(const std::string& device)
{
    auto serialIf = std::make_shared<usb>(device, baud);
    auto [modelseries, modelname] = getmodeltype(serialIf);
    bool match{modelseries == series};
    if (match)
    {
        this->serialIf = serialIf;
        model = modelname;
        scans = initscans(serialIf);
    }
    return match;
}

std::pair<seriesid, std::string>
    Lidar::getmodeltype(std::shared_ptr<serial> serialIf)
{
    seriesid series{seriesid::unknown};
    std::string name;
    serialIf->write({SCANSTARTFLAG, SCANGETINFOCMD});
    std::vector<uint8_t> resp;
    if (serialIf->read(resp, 27))
    {
        uint8_t modelid = resp[7];
        uint8_t majormodelid = (modelid >> 4) & 0x0F;
        uint8_t submodelid = modelid & 0x0F;
        auto getname = [majormodelid, submodelid](char prefix, uint8_t offset) {
            return prefix + std::to_string(majormodelid - offset) + "M" +
                   std::to_string(submodelid);
        };

        if (majormodelid >= (uint8_t)seriesid::mmodel)
        {
            series = seriesid::mmodel;
            name = getname('M', (uint8_t)seriesid::mmodel - 1);
        }
        else if (majormodelid >= (uint8_t)seriesid::tmodel)
        {
            series = seriesid::tmodel;
            name = getname('T', (uint8_t)seriesid::tmodel - 1);
        }
        else if (majormodelid >= (uint8_t)seriesid::smodel)
        {
            series = seriesid::smodel;
            name = getname('S', (uint8_t)seriesid::smodel - 1);
        }
        else if (majormodelid >= (uint8_t)seriesid::cmodel)
        {
            series = seriesid::cmodel;
            name = getname('C', (uint8_t)seriesid::cmodel - 1);
        }
        else
        {
            series = seriesid::amodel;
            name = getname('A', 0);
        }
    }
    return {series, name};
}

void Lidar::watchangle(int32_t angle, const NotifyFunc& notifier)
{
    if (auto scannormal = std::ranges::find_if(
            scans, [](auto scan) { return scan->gettype() == scan_t::normal; });
        scannormal != scans.end())
    {
        (*scannormal)->addangle(angle, notifier);
    }
    if (auto scanexpress = std::ranges::find_if(
            scans,
            [](auto scan) { return scan->gettype() == scan_t::express; });
        scanexpress != scans.end())
    {
        (*scanexpress)->addangle(angle, notifier);
    }
}

std::tuple<std::string, std::string, std::string, std::string> Lidar::getinfo()
{
    serialIf->write({SCANSTARTFLAG, SCANGETINFOCMD});
    std::vector<uint8_t> resp;
    serialIf->read(resp, 27);

    std::ostringstream oss;
    oss << std::showbase << std::hex << (uint32_t)resp[7];
    std::string model{oss.str()};
    oss.str(std::string{});
    oss << std::dec << (uint32_t)resp[9] << "." << (uint32_t)resp[8];
    std::string firmware{oss.str()};
    oss.str(std::string{});
    oss << std::hex << std::showbase << (uint32_t)resp[10];
    std::string hardware{oss.str()};
    oss.str(std::string{});
    for (uint8_t byte = 11; byte < 27; byte++)
    {
        oss << std::hex << std::noshowbase << (uint32_t)resp[byte];
    }
    std::string serialnum{oss.str()};
    return {model, firmware, hardware, serialnum};
}

std::pair<state, std::string> Lidar::getstate()
{
    static const std::unordered_map<state, std::string> status = {
        {state::good, "\e[1;32mOk\e[0m"},
        {state::warn, "\e[1;33mWarning\e[0m"},
        {state::err, "\e[1;31mERROR\e[0m"}};

    serialIf->write({SCANSTARTFLAG, SCANGETSTATCMD});
    std::vector<uint8_t> resp;
    serialIf->read(resp, 10);

    auto code = static_cast<state>(resp.at(7));
    auto name = status.contains(code) ? status.at(code) : std::string{};
    return {code, name};
}

std::pair<uint16_t, uint16_t> Lidar::getsamplerate()
{
    static constexpr auto makeval = [](uint8_t hi, uint8_t low) -> uint16_t {
        return hi << 8 | low;
    };
    serialIf->write({SCANSTARTFLAG, SCANGETSRATECMD});
    std::vector<uint8_t> resp;
    serialIf->read(resp, 11);
    auto normalscantime{makeval(resp[8], resp[7])};
    auto expressscantime{makeval(resp[10], resp[9])};
    return {normalscantime, expressscantime};
}

Configuration Lidar::getconfiguration()
{
    static constexpr uint8_t reqdatasize = 4, respdatasize = reqdatasize,
                             resppacketsize = respdatasize + 7;
    Configuration config;
    std::vector<uint8_t> resp;

    getpacket({SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize, 0x70, 0, 0, 0}, resp,
              resppacketsize + 2, true);
    config.modecnt = resp[12] << 8 | resp[11];

    resp.clear();
    getpacket({SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize, 0x7C, 0, 0, 0}, resp,
              resppacketsize + 2, true);
    config.typical = resp[12] << 8 | resp[11];

    for (uint8_t mode = 0; mode < config.modecnt; mode++)
    {
        resp.clear();
        getpacket({SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize + 2, 0x7F, 0, 0,
                   0, mode, 0},
                  resp, resppacketsize + 200, true);
        std::string name(resp.begin() + 11, resp.end());

        resp.clear();
        getpacket({SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize + 2, 0x71, 0, 0,
                   0, mode, 0},
                  resp, resppacketsize + 4, true);
        uint32_t uscostpersample =
            resp[14] << 24 | resp[13] << 16 | resp[12] << 8 | resp[11];
        uscostpersample /= 256;
        uint32_t maxsamplerate = 1000 * 1000 / uscostpersample;

        resp.clear();
        getpacket({SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize + 2, 0x74, 0, 0,
                   0, mode, 0},
                  resp, resppacketsize + 4, true);
        uint32_t maxdistance =
            resp[14] << 24 | resp[13] << 16 | resp[12] << 8 | resp[11];
        maxdistance /= 256;

        resp.clear();
        getpacket({SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize + 2, 0x75, 0, 0,
                   0, mode, 0},
                  resp, resppacketsize + 1, true);
        uint32_t answercmdtype = resp[11];

        config.modes.emplace_back(mode, name, uscostpersample, maxsamplerate,
                                  maxdistance, answercmdtype);
    }

    return config;
}

void Lidar::runscan(scan_t type)
{
    std::ranges::for_each(scans, [type](auto scan) {
        if (scan->gettype() == type)
        {
            scan->run();
        }
    });
}

void Lidar::stopscan()
{
    std::ranges::for_each(scans, [](auto scan) {
        if (scan->isrunning())
        {
            scan->stop();
        }
    });
}

Lidar::Lidar(seriesid series, speed_t baud, scansinitfunc&& initscans) :
    series{series}, baud{baud}, initscans{std::move(initscans)}
{}

void Lidar::getpacket(std::vector<uint8_t>&& req, std::vector<uint8_t>& resp,
                      uint8_t size, bool chsum = false)
{
    if (chsum)
    {
        req.push_back(getchecksum(req));
    }
    serialIf->write(req);
    serialIf->read(resp, size);
}
