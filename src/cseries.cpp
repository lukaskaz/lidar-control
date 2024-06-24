#include "climenu.hpp"
#include "lidar.hpp"

#include <functional>

void Cseries::run()
{
    Menu("[Lidar " + modelname + " scanner on " + device + " @ " + baud + "]",
         {{"get info", std::bind(&Cseries::readinfo, this)},
          {"get status", std::bind(&Cseries::readstatus, this)},
          {"get sampling time", std::bind(&Cseries::readsamplerate, this)},
          {"get configuration", std::bind(&Cseries::readconfiguration, this)},
          {"start normal scanning", std::bind(&Cseries::readnormalscan, this)},
          {"start express scanning (dense)",
           std::bind(&Cseries::readexpressscan, this)},
          {"exit", [this]() { Cseries::exitprogram(); }}})
        .run();
}
