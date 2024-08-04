#pragma once

#include "interfaces/scan.hpp"
#include "samples.hpp"
#include "serial.hpp"

#include <cstdint>
#include <future>
#include <memory>
#include <vector>

using Measurement = std::pair<bool, SampleData>;

class Scan : public ScanIf
{
  public:
    Scan(std::shared_ptr<serial>, scan_t, scansub_t);
    ~Scan();

    void run() = 0;
    void stop() override;
    void addangle(int32_t, const NotifyFunc&) override;
    void delangle(int32_t) override;

    scan_t gettype() const override;
    std::string getsubtypename() const override;

    bool isrunning() const override;

  protected:
    Observer observer;
    const scan_t type;
    const scansub_t subtype;
    std::atomic<bool> running{false};
    std::shared_ptr<serial> serialIf;
    std::shared_ptr<std::future<void>> scanning;

    virtual void requestscan() = 0;
    virtual void releasescan();
};

class ScanNormal : public Scan, public std::enable_shared_from_this<ScanNormal>
{
  public:
    void run() override;

  private:
    friend class ScanFactory;
    ScanNormal(std::shared_ptr<serial> serialIf) :
        Scan(serialIf, scan_t::normal, scansub_t::none)
    {}
    void requestscan() override;
    Measurement getdata(bool);
};

class ScanExpress : public Scan
{
  public:
    ScanExpress(std::shared_ptr<serial> serialIf, scansub_t subtype) :
        Scan(serialIf, scan_t::express, subtype)
    {}

    void run() = 0;

  protected:
    void requestscan() override;
    virtual std::pair<double, std::vector<uint8_t>> getbasedata(bool);
};

class ScanExpressLegacy :
    public ScanExpress,
    public std::enable_shared_from_this<ScanExpressLegacy>
{
  public:
    void run() override;

  private:
    friend class ScanFactory;
    ScanExpressLegacy(std::shared_ptr<serial> serialIf) :
        ScanExpress(serialIf, scansub_t::legacy)
    {}
    std::array<Measurement, 2> getcabindata(std::vector<uint8_t>&&, double,
                                            double, uint8_t);
};

class ScanExpressDense :
    public ScanExpress,
    public std::enable_shared_from_this<ScanExpressDense>
{
  public:
    void run() override;

  private:
    friend class ScanFactory;
    ScanExpressDense(std::shared_ptr<serial> serialIf) :
        ScanExpress(serialIf, scansub_t::dense)
    {}
    Measurement getcabindata(std::vector<uint8_t>&&, double, double, uint8_t);
};
