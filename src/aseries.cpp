#include "climenu.hpp"
#include "lidar.hpp"

#include <functional>

Aseries::Aseries(std::shared_ptr<serial> serialIf, const std::string& device,
                 const std::string& name) :
    Common(serialIf, device, name)
{
    normalscan = std::make_shared<Normalscan>(serialIf);
    expressscan = std::make_shared<Expresslegacyscan>(serialIf);
}

void Aseries::observe(int32_t angle, const NotifyFunc& notifier)
{
    Common::observe(angle, notifier);
}

void Aseries::run()
{
    Menu("[Lidar " + modelname + " scanner on " + device + " @ " + baud + "]",
         {{"get info", std::bind(&Aseries::readinfo, this)},
          {"get status", std::bind(&Aseries::readstatus, this)},
          {"get sampling time", std::bind(&Aseries::readsamplerate, this)},
          {"get configuration", std::bind(&Aseries::readconfiguration, this)},
          {"start normal scanning", std::bind(&Aseries::runnormalscan, this)},
          {"stop normal scanning", std::bind(&Aseries::stopnormalscan, this)},
          {"start express scanning (legacy)",
           std::bind(&Aseries::runexpressscan, this)},
          {"stop express scanning", std::bind(&Aseries::stopexpressscan, this)},
          {"exit", [this]() { Aseries::exitprogram(); }}})
        .run();
}
