#include "common.hpp"

#include "helpers.hpp"

#include <iostream>
#include <unordered_map>

bool Common::detect(const std::string& device, speed_t baud, seriesid series)
{
    static const std::unordered_map<uint32_t, std::string> speedtostring = {
        {B115200, "115200"}, {B460800, "460800"}};
    this->device = device;
    this->baud = speedtostring.at(baud);
    serialIf = std::make_shared<usb>(device, baud);
    std::tie(modelseries, modelname) = readmodeltype();
    if (modelseries != series)
    {
        serialIf.reset();
        return false;
    }
    return true;
}

std::pair<seriesid, std::string> Common::readmodeltype()
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

void Common::readinfo()
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

void Common::readstatus()
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

void Common::readsamplerate()
{
    serialIf->write({SCANSTARTFLAG, SCANGETSRATECMD});
    std::vector<uint8_t> resp;
    serialIf->read(resp, 11);

    printf("Single std scan time: %u ms\n", (resp[8] << 8) | resp[7]);
    printf("Single express scan time: %u ms\n", (resp[10] << 8) | resp[9]);
}

void Common::readconfiguration()
{
    constexpr uint8_t reqdatasize = 4, respdatasize = reqdatasize,
                      resppacketsize = respdatasize + 7;

    std::vector<uint8_t> resp;
    getpacket({SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize, 0x70, 0, 0, 0}, resp,
              resppacketsize + 2, true);
    uint32_t modescount = resp[12] << 8 | resp[11];
    std::cout << "Scan modes count: " << modescount << "\n";

    resp.clear();
    getpacket({SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize, 0x7C, 0, 0, 0}, resp,
              resppacketsize + 2, true);
    uint32_t typicalmode = resp[12] << 8 | resp[11];
    std::cout << "Typical scan mode: " << typicalmode << "\n";

    for (uint8_t mode = 0; mode < modescount; mode++)
    {
        resp.clear();
        getpacket({SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize + 2, 0x7F, 0, 0,
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
        getpacket({SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize + 2, 0x71, 0, 0,
                   0, mode, 0},
                  resp, resppacketsize + 4, true);

        uint32_t uscostpersample =
            resp[14] << 24 | resp[13] << 16 | resp[12] << 8 | resp[11];
        uscostpersample /= 256;
        std::cout << "\t> cost per sample: " << uscostpersample << " us\n";
        std::cout << "\t> max sample rate: " << 1000 * 1000 / uscostpersample
                  << " sps\n";

        resp.clear();
        getpacket({SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize + 2, 0x74, 0, 0,
                   0, mode, 0},
                  resp, resppacketsize + 4, true);

        uint32_t maxdistance =
            resp[14] << 24 | resp[13] << 16 | resp[12] << 8 | resp[11];
        maxdistance /= 256;
        std::cout << "\t> max distance: " << maxdistance << "m\n";

        resp.clear();
        getpacket({SCANSTARTFLAG, SCANGETCONFCMD, reqdatasize + 2, 0x75, 0, 0,
                   0, mode, 0},
                  resp, resppacketsize + 1, true);

        uint32_t answercmdtype = resp[11];
        std::cout << "\t> answer cmd type: " << std::hex << std::showbase
                  << answercmdtype << std::noshowbase << std::dec << "\n";
        std::cout << "\n";
    }
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
    printf("Cleaning and closing\n");
}
