#pragma once

#include "climenu.hpp"
#include "interfaces/lidar.hpp"

class Display
{
  public:
    Display(std::shared_ptr<LidarIf> lidar) : lidar{lidar}
    {}

    void run();

  private:
    std::shared_ptr<LidarIf> lidar;

    void info();
    void state();
    void samplerate();
    void configuration();
    void scanning(scan_t);
    void exitprogram();
};
