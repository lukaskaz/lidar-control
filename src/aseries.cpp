#include "display.hpp"
#include "lidar.hpp"

Aseries::Aseries(std::shared_ptr<serial> serialIf, const std::string& device,
                 const std::string& name) :
    Common(serialIf, device, name, baud, scantype),
    Display(serialIf)
{
    normalscan = std::make_shared<Normalscan>(serialIf);
    expressscan = std::make_shared<Expresslegacyscan>(serialIf);
}

void Aseries::observe(int32_t angle, const NotifyFunc& notifier)
{
    Common::observe(angle, notifier);
}

void Aseries::menu()
{
    Display::run();
}
