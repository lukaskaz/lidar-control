#include "climenu.hpp"
#include "lidar.hpp"

#include <functional>

Cseries::Cseries(std::shared_ptr<serial> serialIf, const std::string& device,
                 const std::string& name) :
    Common(serialIf, device, name)
{
    normalscan = std::make_shared<Normalscan>(serialIf);
    expressscan = std::make_shared<Expressdensescan>(serialIf);
}

void Cseries::observe(int32_t angle, const NotifyFunc& notifier)
{
    Common::observe(angle, notifier);
}

void Cseries::run()
{
    Menu("[Lidar " + modelname + " scanner on " + device + " @ " + baud + "]",
         {{"get info", std::bind(&Cseries::readinfo, this)},
          {"get status", std::bind(&Cseries::readstatus, this)},
          {"get sampling time", std::bind(&Cseries::readsamplerate, this)},
          {"get configuration", std::bind(&Cseries::readconfiguration, this)},
          {"start normal scanning", std::bind(&Cseries::runnormalscan, this)},
          {"stop normal scanning", std::bind(&Cseries::stopnormalscan, this)},
          {"start express scanning (dense)",
           std::bind(&Cseries::runexpressscan, this)},
          {"stop express scanning", std::bind(&Cseries::stopexpressscan, this)},
          {"exit", [this]() { Cseries::exitprogram(); }}})
        .run();
}
