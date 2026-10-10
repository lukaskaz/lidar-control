#pragma once

#include "lidar/interface/lidar.hpp"
#include "menu/interfaces/cli.hpp"

using namespace Lidar;

class Display
{
  public:
    Display(std::shared_ptr<Interface::Lidar> lidar) : lidar{lidar}
    {}

    void run();

  private:
    std::shared_ptr<Interface::Lidar> lidar;

    bool info();
    bool state();
    bool samplerate();
    bool configuration();
    bool scanning(Scan::scan_t);
    bool exitprogram();
};
