#include "display.hpp"
#include "lidar.hpp"

Cseries::Cseries(std::shared_ptr<serial> serialIf, const std::string& device,
                 const std::string& name) :
    Common(serialIf, device, name, baud, scantype),
    Display(serialIf)
{
    normalscan = std::make_shared<Normalscan>(serialIf);
    expressscan = std::make_shared<Expressdensescan>(serialIf);
}

void Cseries::observe(int32_t angle, const NotifyFunc& notifier)
{
    Common::observe(angle, notifier);
}

void Cseries::menu()
{
    Display::run();
}
