#pragma once

#include "interfaces/lidar.hpp"
#include "menu/interfaces/cli.hpp"

class Display
{
  public:
    Display(std::shared_ptr<LidarIf> lidar) : lidar{lidar}
    {}

    void run();

  private:
    std::shared_ptr<LidarIf> lidar;

    bool info();
    bool state();
    bool samplerate();
    bool configuration();
    bool scanning(scan_t);
    bool exitprogram();
};
