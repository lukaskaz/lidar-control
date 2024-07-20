#pragma once

#include "samples.hpp"
#include "serial.hpp"

#include <cstdint>
#include <future>
#include <map>
#include <memory>
#include <tuple>
#include <vector>

using Measurement = std::pair<bool, SampleData>;

class ScanningIf
{
  public:
    Observer observer;

  public:
    ScanningIf(std::shared_ptr<serial>);
    virtual ~ScanningIf()
    {}
    virtual void run() = 0;
    virtual void stop();
    virtual bool isrunning() const;

  protected:
    std::atomic<bool> running{false};
    std::shared_ptr<serial> serialIf;
    std::shared_ptr<std::future<void>> scanning;

    virtual void requestscan() = 0;
    virtual void releasescan();
};

class Normalscan :
    public ScanningIf,
    public std::enable_shared_from_this<Normalscan>
{
  public:
    explicit Normalscan(std::shared_ptr<serial> serialIf) : ScanningIf(serialIf)
    {}
    ~Normalscan()
    {
        stop();
    }

    void run() override;
    void stop() override;

  private:
    void requestscan() override;
    Measurement getdata(bool);
};

class ExpressscanIf : public ScanningIf
{
  public:
    ExpressscanIf(std::shared_ptr<serial> serialIf) : ScanningIf(serialIf)
    {}
    virtual ~ExpressscanIf()
    {}
    void requestscan();
    virtual std::pair<double, std::vector<uint8_t>> getbasedata(bool);
};

class Expresslegacyscan :
    public ExpressscanIf,
    public std::enable_shared_from_this<Expresslegacyscan>
{
  public:
    explicit Expresslegacyscan(std::shared_ptr<serial> serialIf) :
        ExpressscanIf(serialIf)
    {}
    ~Expresslegacyscan()
    {
        stop();
    }

    void run() override;
    void stop() override;

  private:
    std::array<Measurement, 2> getcabindata(std::vector<uint8_t>&&, double,
                                            double, uint8_t);
};

class Expressdensescan :
    public ExpressscanIf,
    public std::enable_shared_from_this<Expressdensescan>
{
  public:
    explicit Expressdensescan(std::shared_ptr<serial> serialIf) :
        ExpressscanIf(serialIf)
    {}
    ~Expressdensescan()
    {
        stop();
    }

    void run() override;
    void stop() override;

  private:
    Measurement getcabindata(std::vector<uint8_t>&&, double, double, uint8_t);
};
