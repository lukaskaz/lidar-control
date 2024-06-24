#pragma once

#include "common.hpp"

#include <algorithm>
#include <stdexcept>

class LidarIf
{
  public:
    virtual ~LidarIf()
    {}
    virtual bool detect(const std::string&) = 0;
    virtual void run() = 0;
};

class Aseries : public LidarIf, public Common
{
  public:
    Aseries()
    {}
    ~Aseries()
    {}
    void run() override;
    bool detect(const std::string& device) override
    {
        return Common::detect(device, B115200, seriesid::amodel);
    }
    void readexpressscan() override
    {
        Expressscan(serialIf).runlegacy();
    }
};

class Cseries : public LidarIf, public Common
{
  public:
    Cseries()
    {}
    ~Cseries()
    {}
    void run() override;
    bool detect(const std::string& device) override
    {
        return Common::detect(device, B460800, seriesid::cmodel);
    }
    void readexpressscan() override
    {
        Expressscan(serialIf).rundense();
    }
};

class Lidar
{
  public:
    static std::shared_ptr<LidarIf> detect(const std::string& device)
    {
        static const std::vector<std::shared_ptr<LidarIf>> lidars = {
            std::make_shared<Aseries>(), std::make_shared<Cseries>()};

        if (auto detected = std::ranges::find_if(
                lidars, [device](auto lidar) { return lidar->detect(device); });
            detected != lidars.end())
        {
            return *detected;
        }
        throw std::runtime_error("Cannot detect lidar");
    }
};
