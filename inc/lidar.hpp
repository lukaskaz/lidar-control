#pragma once

#include "common.hpp"

#include <algorithm>
#include <stdexcept>

class LidarIf
{
  public:
    virtual ~LidarIf()
    {}
    virtual void observe(int32_t, const NotifyFunc&) = 0;
    virtual void run() = 0;
};

class Aseries : public LidarIf, public Common
{
  public:
    static constexpr std::string baud{"115200"};
    static constexpr speed_t speed{B115200};
    static constexpr seriesid series{seriesid::amodel};

  public:
    Aseries(std::shared_ptr<serial>, const std::string&, const std::string&);
    ~Aseries()
    {}
    void observe(int32_t, const NotifyFunc&) override;
    void run() override;
};

class Cseries : public LidarIf, public Common
{
  public:
    static constexpr std::string baud{"460800"};
    static constexpr speed_t speed{B460800};
    static constexpr seriesid series{seriesid::cmodel};

  public:
    Cseries(std::shared_ptr<serial>, const std::string&, const std::string&);
    ~Cseries()
    {}

    void observe(int32_t, const NotifyFunc&) override;
    void run() override;
};

class LidarFactoryIf
{
  public:
    virtual ~LidarFactoryIf()
    {}
    virtual bool detect() = 0;
    virtual std::shared_ptr<LidarIf> create() = 0;
};

template <typename T>
    requires requires(T t, std::shared_ptr<serial> serial, seriesid series) {
        {
            t.baud
        } -> std::same_as<const std::string&>;
        {
            t.speed
        } -> std::same_as<const speed_t&>;
        {
            t.series
        } -> std::same_as<const seriesid&>;
        {
            t.detect(serial, series)
        } -> std::same_as<std::pair<bool, std::string>>;
    }
class LidarFactory : public LidarFactoryIf
{
  public:
    explicit LidarFactory(const std::string& device) : device{device}
    {}
    LidarFactory(const LidarFactory&) = delete;
    LidarFactory(LidarFactory&&) = delete;
    LidarFactory& operator=(const LidarFactory&) = delete;
    LidarFactory& operator=(LidarFactory&&) = delete;

    bool detect() override
    {
        bool detected{};
        auto serialIf = std::make_shared<usb>(device, T::speed);
        std::tie(detected, modulename) = T::detect(serialIf, T::series);
        return detected;
    }

    std::shared_ptr<LidarIf> create() override
    {
        auto serialIf = std::make_shared<usb>(device, T::speed);
        return std::make_shared<T>(serialIf, device, modulename);
    }

  private:
    const std::string device;
    std::string modulename;
};

class Lidar
{
  public:
    static std::shared_ptr<LidarIf> detect(const std::string& device)
    {
        static const std::vector<std::shared_ptr<LidarFactoryIf>> factories = {
            std::make_shared<LidarFactory<Aseries>>(device),
            std::make_shared<LidarFactory<Cseries>>(device)};

        if (auto detected = std::ranges::find_if(
                factories, [](auto factory) { return factory->detect(); });
            detected != factories.end())
        {
            return (*detected)->create();
        }
        throw std::runtime_error("Cannot detect lidar");
    }
};
