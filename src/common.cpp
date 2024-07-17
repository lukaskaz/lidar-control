#include "common.hpp"

#include "helpers.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <unordered_map>

Common::Common(std::shared_ptr<serial> serialIf, const std::string& device,
               const std::string& name) :
    serialIf{serialIf},
    device{device}, modelname{name}
{}

std::pair<bool, std::string> Common::detect(std::shared_ptr<serial> serialIf,
                                            seriesid series)
{
    auto [modelseries, modelname] = readmodeltype(serialIf);
    return {modelseries == series, modelname};
}

std::pair<seriesid, std::string>
    Common::readmodeltype(std::shared_ptr<serial> serialIf)
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

void Common::readinfo()
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

    std::cout << "Model: " << model << "\n";
    std::cout << "Firmware: " << firmware << "\n";
    std::cout << "Hardware: " << hardware << "\n";
    std::cout << "Serialnum: " << serialnum << "\n";
}

void Common::readstatus()
{
    enum class state
    {
        good,
        warn,
        err
    };
    static const std::unordered_map<state, std::string> status = {
        {state::good, "\e[1;32mOk\e[0m"},
        {state::warn, "\e[1;33mWarning\e[0m"},
        {state::err, "\e[1;31mERROR\e[0m"}};

    serialIf->write({SCANSTARTFLAG, SCANGETSTATCMD});
    std::vector<uint8_t> resp;
    serialIf->read(resp, 10);

    state currstate = static_cast<state>(resp.at(7));
    if (status.contains(currstate))
    {
        std::cout << "Current status: " << status.at(currstate) << "\n";
    }
    else
    {
        std::cerr << "Cannot recognize status: " << (uint32_t)currstate << "\n";
    }
}

void Common::readsamplerate()
{
    static constexpr auto makeval = [](uint8_t hi, uint8_t low) -> uint16_t {
        return hi << 8 | low;
    };
    serialIf->write({SCANSTARTFLAG, SCANGETSRATECMD});
    std::vector<uint8_t> resp;
    serialIf->read(resp, 11);
    auto normalscantime{makeval(resp[8], resp[7])};
    auto expressscantime{makeval(resp[10], resp[9])};

    std::cout << "Normal scan: " << normalscantime << "ms\n";
    std::cout << "Express scan: " << expressscantime << "ms\n";
}

void Common::readconfiguration()
{
    constexpr uint8_t reqdatasize = 4, respdatasize = reqdatasize,
                      resppacketsize = respdatasize + 7;

    struct Config
    {
        struct Mode
        {
            uint32_t id;
            std::string name;
            uint32_t uscostpersample;
            uint32_t maxsamplerate;
            uint32_t maxdistance;
            uint32_t answercmdtype;
        };
        uint32_t modecnt;
        uint32_t typical;
        std::vector<Mode> modes;
    } config;

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

    std::cout << "Scan modes count: " << config.modecnt << "\n";
    std::cout << "Typical scan mode: " << config.typical << "\n";
    std::ranges::for_each(config.modes, [typical{config.typical}](
                                            const auto& mode) {
        std::cout << "Info for mode " << mode.id << " [" << mode.name << "]";
        if (mode.id == typical)
        {
            std::cout << " -> TYPICAL";
        }
        std::cout << "\n";
        std::cout << "\t> cost per sample: " << mode.uscostpersample << " us\n";
        std::cout << "\t> max sample rate: " << mode.maxsamplerate << " sps\n";
        std::cout << "\t> max distance: " << mode.maxdistance << "m\n";
        std::cout << "\t> answer cmd type: " << std::hex << std::showbase
                  << mode.answercmdtype << std::noshowbase << std::dec << "\n";
        std::cout << "\n";
    });
}

void Common::runnormalscan()
{
    if (expressscan->isrunning())
    {
        expressscan->stop();
    }
    normalscan->run();
}

void Common::stopnormalscan()
{
    normalscan->stop();
}

void Common::runexpressscan()
{
    if (normalscan->isrunning())
    {
        normalscan->stop();
    }
    expressscan->run();
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

void Common::exitprogram()
{
    std::cout << "Cleaning and closing\n";
}
