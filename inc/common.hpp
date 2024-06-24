#pragma once

#include "scanning.hpp"
#include "serial.hpp"

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#define MODULE_NAME "Lidar 360 scanner"

#define SCANSTARTFLAG 0xA5
#define SCANGETINFOCMD 0x50
#define SCANGETSTATCMD 0x52
#define SCANGETSRATECMD 0x59
#define SCANGETCONFCMD 0x84

enum class seriesid
{
    amodel = 0,
    cmodel = 4,
    smodel = 6,
    tmodel = 9,
    mmodel = 12,
    unknown = 255
};

class Common
{
  public:
    explicit Common()
    {}
    virtual ~Common(){
        // stopscanning();
    };

  protected:
    bool detect(const std::string&, speed_t, seriesid);

    virtual void readinfo();
    virtual void readstatus();
    virtual void readsamplerate();
    virtual void readconfiguration();

    virtual void readnormalscan()
    {
        Normalscan(serialIf).run();
    }
    virtual void readexpressscan() = 0;

    virtual void exitprogram();
    virtual std::pair<seriesid, std::string> readmodeltype();
    void getpacket(std::vector<uint8_t>&&, std::vector<uint8_t>&, uint8_t,
                   bool);

  protected:
    std::string device;
    std::string baud;
    seriesid modelseries;
    std::string modelname;
    std::shared_ptr<serial> serialIf;
};
