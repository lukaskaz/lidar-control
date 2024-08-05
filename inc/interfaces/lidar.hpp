#pragma once

#include "interfaces/scan.hpp"
#include "serial.hpp"

#include <memory>

enum class state
{
    good,
    warn,
    err
};

enum class seriesid
{
    amodel = 0,
    cmodel = 4,
    smodel = 6,
    tmodel = 9,
    mmodel = 12,
    unknown = 255
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

class LidarInfoIf
{
  public:
    virtual ~LidarInfoIf()
    {}

    virtual seriesid getseries(std::shared_ptr<serial>) = 0;
    virtual std::string getname() = 0;
    virtual std::tuple<std::string, std::string, std::string, std::string>
        getfwinfo() = 0;
    virtual std::pair<state, std::string> getstate() = 0;
    virtual std::pair<uint16_t, uint16_t> getsamplerate() = 0;
    virtual Configuration getconfiguration() = 0;
    virtual std::pair<std::string, std::string> getconninfo() = 0;
};

class LidarScanIf
{
  public:
    virtual ~LidarScanIf()
    {}

    virtual void watchangle(int32_t, const NotifyFunc&) = 0;
    virtual void runscan(scan_t) = 0;
    virtual void stopscan() = 0;
    virtual std::pair<std::string, std::string> getscaninfo(scan_t) const = 0;
};

class LidarIf : public LidarInfoIf, public LidarScanIf
{
  public:
    virtual ~LidarIf()
    {}

    virtual bool setup(const std::string&) = 0;
};
