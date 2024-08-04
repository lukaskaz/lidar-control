#pragma once

#include "climenu.hpp"
#include "interfaces/lidar.hpp"

class Display
{
  public:
    Display(LidarIf& lidar, std::vector<std::shared_ptr<ScanIf>>& scans) :
        lidar{lidar}, scans{scans}
    {}

    void run(std::tuple<std::string, std::string, std::string, std::string>&&);

  private:
    LidarIf& lidar;
    std::vector<std::shared_ptr<ScanIf>>& scans;

    void info();
    void state();
    void samplerate();
    void configuration();
    void normalscanning();
    void expressscanning(const std::string& type);
    void exitprogram();
};
