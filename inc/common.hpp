#pragma once

#include "scanning.hpp"
#include "serial.hpp"

#include <cstdint>
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

enum class state
{
    good,
    warn,
    err
};

struct Configuration
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
};

class Common
{
  public:
    virtual ~Common(){};
    explicit Common(std::shared_ptr<serial>, const std::string&,
                    const std::string&, const std::string&, const std::string&);

    static std::pair<bool, std::string> detect(std::shared_ptr<serial>,
                                               seriesid);

  protected:
    static std::pair<seriesid, std::string>
        getmodeltype(std::shared_ptr<serial>);

    virtual void observe(int32_t, const NotifyFunc&);
    virtual std::tuple<std::string, std::string, std::string, std::string>
        getinfo();
    virtual std::pair<state, std::string> getstate();
    virtual std::pair<uint16_t, uint16_t> getsamplerate();
    virtual Configuration getconfiguration();

    virtual void runnormalscan();
    virtual void stopnormalscan();

    virtual void runexpressscan();
    virtual void stopexpressscan();

    void getpacket(std::vector<uint8_t>&&, std::vector<uint8_t>&, uint8_t,
                   bool);

  protected:
    std::shared_ptr<serial> serialIf;
    std::string device;
    std::string modelname;
    std::string baud;
    std::string expressscantype;
    std::shared_ptr<ScanningIf> normalscan;
    std::shared_ptr<ScanningIf> expressscan;
};

void statustest(Observer&);
