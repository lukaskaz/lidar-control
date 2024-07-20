#include "common.hpp"

#include "helpers.hpp"

#include <algorithm>
#include <sstream>
#include <unordered_map>

Common::Common(std::shared_ptr<serial> serialIf, const std::string& device,
               const std::string& name, const std::string& baud,
               const std::string& scantype) :
    serialIf{serialIf},
    device{device}, modelname{name}, baud{baud}, expressscantype{scantype}
{}

std::pair<bool, std::string> Common::detect(std::shared_ptr<serial> serialIf,
                                            seriesid series)
{
    auto [modelseries, modelname] = getmodeltype(serialIf);
    return {modelseries == series, modelname};
}

std::pair<seriesid, std::string>
    Common::getmodeltype(std::shared_ptr<serial> serialIf)
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

void Common::observe(int32_t angle, const NotifyFunc& notifier)
{
    normalscan->observer.event(angle, notifier);
    expressscan->observer.event(angle, notifier);
}

std::tuple<std::string, std::string, std::string, std::string> Common::getinfo()
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

std::pair<state, std::string> Common::getstate()
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

std::pair<uint16_t, uint16_t> Common::getsamplerate()
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

Configuration Common::getconfiguration()
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

void Common::runnormalscan()
{
    if (!expressscan->isrunning())
    {
        normalscan->run();
    }
}

void Common::stopnormalscan()
{
    normalscan->stop();
}

void Common::runexpressscan()
{
    if (!normalscan->isrunning())
    {
        expressscan->run();
    }
}

void Common::stopexpressscan()
{
    expressscan->stop();
}

void Common::getpacket(std::vector<uint8_t>&& req, std::vector<uint8_t>& resp,
                       uint8_t size, bool chsum = false)
{
    if (chsum)
    {
        req.push_back(getchecksum(req));
    }
    serialIf->write(req);
    serialIf->read(resp, size);
}
