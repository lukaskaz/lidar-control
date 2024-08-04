#pragma once

#include "interfaces/scan.hpp"
#include "serial.hpp"

#include <memory>

class ScanFactory
{
  public:
    static std::shared_ptr<ScanIf> createNormal(std::shared_ptr<serial>);
    static std::shared_ptr<ScanIf> createExpressLegacy(std::shared_ptr<serial>);
    static std::shared_ptr<ScanIf> createExpressDense(std::shared_ptr<serial>);
};
