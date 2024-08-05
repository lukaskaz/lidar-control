#pragma once

#include "interfaces/lidar.hpp"

class Lidar : public LidarIf
{
  public:
    bool setup(const std::string&) override;

    seriesid getseries(std::shared_ptr<serial>) override;
    std::string getname() override;
    std::tuple<std::string, std::string, std::string, std::string>
        getfwinfo() override;
    std::pair<state, std::string> getstate() override;
    std::pair<uint16_t, uint16_t> getsamplerate() override;
    Configuration getconfiguration() override;
    std::pair<std::string, std::string> getconninfo() override;

    void watchangle(int32_t, const NotifyFunc&) override;
    void runscan(scan_t) override;
    void stopscan() override;
    std::pair<std::string, std::string> getscaninfo(scan_t) const override;

  protected:
    using scansinittype = std::vector<std::shared_ptr<ScanIf>>;
    using scansinitfunc = std::function<scansinittype(std::shared_ptr<serial>)>;
    const seriesid series;
    const speed_t baud;
    const scansinitfunc initscans;
    std::shared_ptr<serial> serialIf;
    std::vector<std::shared_ptr<ScanIf>> scans;

  private:
    friend class LidarFactory;
    Lidar(seriesid, speed_t, scansinitfunc&&);

    void getpacket(std::vector<uint8_t>&&, std::vector<uint8_t>&, uint8_t,
                   bool);
    std::shared_ptr<ScanIf> getscan(scan_t) const;
};
