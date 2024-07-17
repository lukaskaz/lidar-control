#include "samples.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#define MAXANGLEPERSCAN 360

Sample::Sample(int32_t angle,
               std::shared_ptr<std::vector<NotifyFunc>> notifiers) :
    angle{angle},
    distance{getinvalid()}, notifiers{notifiers}
{}

void Sample::reset()
{
    distance = getinvalid();
}

void Sample::update(double value)
{
    distance = value < invaliddistance ? getinvalid() : value;
    if (isvalid())
    {
        std::ranges::for_each(
            *notifiers, [this](NotifyFunc& notifier) { notifier(get()); });
    }
}

SampleData Sample::get() const
{
    return {angle, distance};
}

bool Sample::isvalid() const
{
    return !std::isnan(distance);
}

double Sample::getinvalid() const
{
    return std::numeric_limits<decltype(distance)>::quiet_NaN();
}

SampleMonitor::SampleMonitor(int32_t angle) :
    SampleMonitor(angle, defaultsupportnum)
{}

const int32_t SampleMonitor::defaultsupportnum{2};

SampleMonitor::SampleMonitor(int32_t angle, int32_t supportnum)
{
    samples.emplace_back(angle, notifiers);
    for (int32_t num{1}; num < supportnum; num += 2)
    {
        auto prev =
            angle - num < 0 ? MAXANGLEPERSCAN + angle - num : angle - num;
        auto next = angle + num >= MAXANGLEPERSCAN
                        ? angle + num - MAXANGLEPERSCAN
                        : angle + num;
        samples.emplace_back(prev, notifiers);
        samples.emplace_back(next, notifiers);
    }
}

void SampleMonitor::addnotifier(NotifyFunc&& func)
{
    notifiers->push_back(std::move(func));
}

std::vector<Sample>& SampleMonitor::get()
{
    return samples;
}

Observer::Observer()
{}

void Observer::event(int32_t angle, NotifyFunc func)
{
    auto [it, ret] = samples.emplace(angle, SampleMonitor(angle));
    it->second.addnotifier(std::move(func));
    if (ret)
    {
        std::ranges::for_each(it->second.get(), [this](auto& sample) {
            updater.try_emplace(std::get<int32_t>(sample.get()),
                                std::bind(&Sample::update, std::ref(sample),
                                          std::placeholders::_1));
        });
    }
}

void Observer::update(const SampleData& data)
{
    auto [angle, distance] = data;
    if (updater.contains(angle))
    {
        updater.at(angle)(distance);
    }
}
