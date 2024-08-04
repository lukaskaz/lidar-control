#include "scanfactory.hpp"

#include "scan.hpp"

std::shared_ptr<ScanIf>
    ScanFactory::createNormal(std::shared_ptr<serial> serialIf)
{
    return std::shared_ptr<ScanNormal>(new ScanNormal(serialIf));
}

std::shared_ptr<ScanIf>
    ScanFactory::createExpressLegacy(std::shared_ptr<serial> serialIf)
{
    return std::shared_ptr<ScanExpressLegacy>(new ScanExpressLegacy(serialIf));
}

std::shared_ptr<ScanIf>
    ScanFactory::createExpressDense(std::shared_ptr<serial> serialIf)
{
    return std::shared_ptr<ScanExpressDense>(new ScanExpressDense(serialIf));
}
