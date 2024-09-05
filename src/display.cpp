#include "display.hpp"

#include "helpers.hpp"

#include <algorithm>
#include <functional>
#include <iomanip>
#include <iostream>

void show(const SampleData& data, uint32_t pos)
{
    static constexpr uint32_t initpos{3};

    uint32_t idx = pos + 1;
    auto [angle, distance] = data;
    std::cout << "\e[" << initpos + pos << ";1H[" << std::setfill('0')
              << std::setw(2) << idx << "] angle(dgr) \e[4m"
              << std::setfill('0') << std::setw(3) << angle
              << "\e[0m, dist(cm): \e[4m" << std::setfill('0') << std::setw(6)
              << std::setprecision(1) << std::fixed << distance << "\e[0m\n";
}

void statustest(std::shared_ptr<LidarIf> lidar)
{
    int32_t line{0}, lines{12};
    for (auto angle{0}, last{359}; angle <= last; angle += 360 / lines)
    {
        lidar->watchangle(
            angle, Observer<SampleData>::create(
                       [line](const SampleData& data) { show(data, line); }));
        line++;
    }

    static constexpr uint32_t initpos{5};
    lidar->watchangle(
        180, Observer<SampleData>::create([line{initpos + line + 1}](
                                              const SampleData& data) {
            const auto& [angle, distance] = data;
            auto dist{static_cast<uint32_t>(distance)};
            std::cout << "\e[" << line << ";1H\r\e[K" << std::flush;
            if (distance < 30)
            {
                std::cout << "[" << std::setfill('0') << std::setw(3) << angle
                          << "dgr@" << std::setfill('0') << std::setw(3) << dist
                          << "cm] CRITICAL: OBSTACLE TOO CLOSE\n";
            }
            else if (distance < 60)
            {
                std::cout << "[" << std::setfill('0') << std::setw(3) << angle
                          << "dgr@" << std::setfill('0') << std::setw(3) << dist
                          << "cm] WARNING: OBSTACLE NEARBY\n";
            }
            else
            {
                std::cout << "[" << std::setfill('0') << std::setw(3) << angle
                          << "dgr@" << std::setfill('0') << std::setw(3) << dist
                          << "cm] GOOD: OBSTACLE FAR AWAY\n";
            }
        }));
}

void Display::info()
{
    auto [model, firmware, hardware, serialnum] = lidar->getfwinfo();
    std::cout << "Model: " << model << "\n";
    std::cout << "Firmware: " << firmware << "\n";
    std::cout << "Hardware: " << hardware << "\n";
    std::cout << "Serialnum: " << serialnum << "\n";
}

void Display::state()
{
    auto [code, name] = lidar->getstate();
    std::cout << "Current status: " << std::quoted(name) << " ["
              << (uint32_t)code << "]\n";
}

void Display::samplerate()
{
    auto [normalms, expressms] = lidar->getsamplerate();
    std::cout << "Normal scan: " << normalms << "ms\n";
    std::cout << "Express scan: " << expressms << "ms\n";
}

void Display::configuration()
{
    auto config = lidar->getconfiguration();

    std::cout << "Scan modes count: " << config.modecnt << "\n";
    std::cout << "Typical scan mode: " << config.typical << "\n";
    std::ranges::for_each(config.modes, [typical{config.typical}](
                                            const auto& mode) {
        std::cout << "Info for mode " << mode.id << " [" << mode.name << "]";
        if (mode.id == typical)
        {
            std::cout << " -> TYPICAL";
        }
        std::cout << "\n";
        std::cout << "\t> cost per sample: " << mode.uscostpersample << "us\n ";
        std::cout << "\t> max sample rate :" << mode.maxsamplerate << " sps\n";
        std::cout << "\t> max distance: " << mode.maxdistance << "m\n";
        std::cout << "\t> answer cmd type: " << std::hex << std::showbase
                  << mode.answercmdtype << std::noshowbase << std::dec << "\n";
        std::cout << "\n";
    });
}

void Display::scanning(scan_t type)
{
    auto [name, subname] = lidar->getscaninfo(type);
    name[0] = (char)toupper(name[0]);
    system("clear");
    std::cout << name << " 360";
    if (!subname.empty())
    {
        std::cout << " " << std::quoted(subname);
    }
    std::cout << " scan started @ " << gettimestr();
    std::cout << "\e[?25l\n"; // hide cursor
    statustest(lidar);
    lidar->runscan(type);
    getchar();
    lidar->stopscan();
    system("clear");
    std::cout << name << " 360 scan completed";
    std::cout << "\e[?25h\n"; // show cursor
}

void Display::exitprogram()
{
    std::cout << "Cleaning and closing\n";
}

void Display::run()
{
    const auto& [conndevice, connspeed] = lidar->getconninfo();
    auto title = "[Lidar " + lidar->getname() + " scanner on " + conndevice +
                 " @ " + connspeed + "]";
    std::vector<std::pair<std::string, func>> entries;

    entries.emplace_back("get info", std::bind(&Display::info, this));
    entries.emplace_back("get status", std::bind(&Display::state, this));
    entries.emplace_back("get sampling time",
                         std::bind(&Display::samplerate, this));
    entries.emplace_back("get configuration",
                         std::bind(&Display::configuration, this));
    entries.emplace_back("run normal scanning",
                         std::bind(&Display::scanning, this, scan_t::normal));
    entries.emplace_back("run express scanning [" +
                             std::get<1>(lidar->getscaninfo(scan_t::express)) +
                             "]",
                         std::bind(&Display::scanning, this, scan_t::express));
    entries.emplace_back("exit program", [this]() { exitprogram(); });

    Menu(title, std::move(entries)).run();
}
