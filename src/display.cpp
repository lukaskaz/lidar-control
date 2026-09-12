#include "display.hpp"

#include "helpers.hpp"

#include <algorithm>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>

void show(const SampleData& data, uint32_t pos, uint32_t idx)
{
    auto [angle, distance] = data;
    std::cout << "\e[" << pos << ";1H[" << std::setfill('0') << std::setw(2)
              << idx << "] angle(dgr) \e[4m" << std::setfill('0')
              << std::setw(3) << angle << "\e[0m, dist(cm): \e[4m"
              << std::setfill('0') << std::setw(6) << std::setprecision(1)
              << std::fixed << distance << "\e[0m\n"
              << std::flush;
}

void statustest(std::shared_ptr<LidarIf> lidar)
{
    static std::mutex mtx;
    static constexpr uint32_t measurespos{3};
    static constexpr uint32_t warningspace{2};

    int32_t line{measurespos}, lines{12};
    for (auto angle{0}, last{359}; angle <= last; angle += 360 / lines)
    {
        lidar->watchangle(
            angle, Observer<SampleData>::create([line](const SampleData& data) {
                std::lock_guard<std::mutex> lock(mtx);
                show(data, line, line - measurespos + 1);
            }));
        line++;
    }

    // static constexpr uint32_t initpos{5};
    // lidar->watchangle(
    //     180, Observer<SampleData>::create([line{initpos + line + 1}](
    //                                           const SampleData& data) {
    //         const auto& [angle, distance] = data;
    //         auto dist{static_cast<uint32_t>(distance)};
    //         std::cout << "\e[" << line << ";1H\r\e[K" << std::flush;
    //         if (distance < 30)
    //         {
    //             std::cout << "[" << std::setfill('0') << std::setw(3) <<
    //             angle
    //                       << "dgr@" << std::setfill('0') << std::setw(3) <<
    //                       dist
    //                       << "cm] CRITICAL: OBSTACLE TOO CLOSE\n";
    //         }
    //         else if (distance < 60)
    //         {
    //             std::cout << "[" << std::setfill('0') << std::setw(3) <<
    //             angle
    //                       << "dgr@" << std::setfill('0') << std::setw(3) <<
    //                       dist
    //                       << "cm] WARNING: OBSTACLE NEARBY\n";
    //         }
    //         else
    //         {
    //             std::cout << "[" << std::setfill('0') << std::setw(3) <<
    //             angle
    //                       << "dgr@" << std::setfill('0') << std::setw(3) <<
    //                       dist
    //                       << "cm] GOOD: OBSTACLE FAR AWAY\n";
    //         }
    //     }));

    line += warningspace;
    std::cout << "\e[" << line << ";1H[" << "WARNINGS" << "] \e[4m"
              << std::flush;

    line++;
    lidar->watchangle(
        0, Observer<SampleData>::create([line](const SampleData& data) {
            std::lock_guard<std::mutex> lock(mtx);
            const auto& [angle, distance] = data;
            uint32_t dist{static_cast<uint32_t>(distance)};
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

    line++;
    lidar->watchangle(
        180, Observer<SampleData>::create([line](const SampleData& data) {
            std::lock_guard<std::mutex> lock(mtx);
            const auto& [angle, distance] = data;
            uint32_t dist{static_cast<uint32_t>(distance)};
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

bool Display::info()
{
    auto [model, firmware, hardware, serialnum] = lidar->getfwinfo();
    std::cout << "Model: " << model << "\n";
    std::cout << "Firmware: " << firmware << "\n";
    std::cout << "Hardware: " << hardware << "\n";
    std::cout << "Serialnum: " << serialnum << "\n";
    return true;
}

bool Display::state()
{
    auto [code, name] = lidar->getstate();
    std::cout << "Current status: " << std::quoted(name) << " ["
              << (uint32_t)code << "]\n";
    return true;
}

bool Display::samplerate()
{
    auto [normalms, expressms] = lidar->getsamplerate();
    std::cout << "Normal scan: " << normalms << "ms\n";
    std::cout << "Express scan: " << expressms << "ms\n";
    return true;
}

bool Display::configuration()
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
    return true;
}

bool Display::scanning(scan_t type)
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
    return true;
}

bool Display::exitprogram()
{
    std::cout << "Cleaning and closing\n";
    return true;
}

void Display::run()
{
    const auto& [conndevice, connspeed] = lidar->getconninfo();
    auto title = "[Lidar " + lidar->getname() + " scanner on " + conndevice +
                 " @ " + connspeed + "]";
    std::vector<
        std::tuple<std::string, std::function<bool()>, std::function<bool()>>>
        entries;

    entries.emplace_back(
        "get info", []() { return true; }, std::bind(&Display::info, this));
    entries.emplace_back(
        "get status", []() { return true; }, std::bind(&Display::state, this));
    entries.emplace_back(
        "get sampling time", []() { return true; },
        std::bind(&Display::samplerate, this));
    entries.emplace_back(
        "get configuration", []() { return true; },
        std::bind(&Display::configuration, this));
    entries.emplace_back(
        "run normal scanning", []() { return true; },
        std::bind(&Display::scanning, this, scan_t::normal));
    entries.emplace_back(
        "run express scanning [" +
            std::get<1>(lidar->getscaninfo(scan_t::express)) + "]",
        []() { return true; },
        std::bind(&Display::scanning, this, scan_t::express));
    entries.emplace_back(
        "exit program", []() { return true; },
        [this]() { return exitprogram(); });

    auto menu =
        menu::MenuFactory::create<menu::cli::Menu>(title, std::move(entries));
    menu->run();
}
