#include "climenu.hpp"
#include "lidar.hpp"

#include <functional>

void Aseries::run()
{
    Menu("[Lidar " + modelname + " scanner on " + device + " @ " + baud + "]",
         {{"get info", std::bind(&Aseries::readinfo, this)},
          {"get status", std::bind(&Aseries::readstatus, this)},
          {"get sampling time", std::bind(&Aseries::readsamplerate, this)},
          {"get configuration", std::bind(&Aseries::readconfiguration, this)},
          {"start normal scanning", std::bind(&Aseries::readnormalscan, this)},
          {"start express scanning (legacy)",
           std::bind(&Aseries::readexpressscan, this)},
          {"exit", [this]() { Aseries::exitprogram(); }}})
        .run();
}
