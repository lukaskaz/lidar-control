#pragma once

#include "interfaces/lidar.hpp"

class Lidar : public LidarIf
{
  public:
    void menu() override;
    bool setup(const std::string&) override;

    std::pair<seriesid, std::string>
        getmodeltype(std::shared_ptr<serial>) override;
    std::tuple<std::string, std::string, std::string, std::string>
        getinfo() override;
    std::pair<state, std::string> getstate() override;
    std::pair<uint16_t, uint16_t> getsamplerate() override;
    Configuration getconfiguration() override;

    void watchangle(int32_t, const NotifyFunc&) override;
    void runscan(scan_t) override;
    void stopscan() override;

  protected:
    using scansinittype = std::vector<std::shared_ptr<ScanIf>>;
    using scansinitfunc = std::function<scansinittype(std::shared_ptr<serial>)>;
    const seriesid series;
    speed_t baud;
    const scansinitfunc initscans;
    std::string model;
    std::shared_ptr<serial> serialIf;
    std::vector<std::shared_ptr<ScanIf>> scans;

  private:
    friend class LidarFactory;
    Lidar(seriesid, speed_t, scansinitfunc&&);

    void getpacket(std::vector<uint8_t>&&, std::vector<uint8_t>&, uint8_t,
                   bool);
};
