#pragma once

#include "climenu.hpp"
#include "common.hpp"

class Display : public virtual Common
{
  public:
    Display(std::shared_ptr<serial> serialIf) : Common(serialIf, {}, {}, {}, {})
    {}

    void run();

  private:
    void info();
    void state();
    void samplerate();
    void configuration();
    void normalscanning();
    void expressscanning();
    void exitprogram();
};
