#pragma once

#include "interfaces/samples.hpp"

#include <cstdint>
#include <string>

enum class scan_t
{
    normal,
    express
};

enum class scansub_t
{
    none,
    legacy,
    dense
};

class ScanIf
{
  public:
    virtual ~ScanIf()
    {}
    virtual void run() = 0;
    virtual void stop() = 0;
    virtual void addangle(int32_t, const NotifyFunc&) = 0;
    virtual void delangle(int32_t) = 0;
    virtual scan_t gettype() const = 0;
    virtual std::string getsubtypename() const = 0;
    virtual bool isrunning() const = 0;
};
