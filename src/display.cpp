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

void statustest(Observer& obs)
{
    int32_t line{0}, lines{12};
    for (auto angle{0}, last{359}; angle <= last; angle += 360 / lines)
    {
        obs.event(angle, [line](const SampleData& data) { show(data, line); });
        line++;
    }

    static constexpr uint32_t initpos{5};
    obs.event(180, [line{initpos + line + 1}](const SampleData& data) {
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
    });
}

void Display::info()
{
    auto [model, firmware, hardware, serialnum] = getinfo();
    std::cout << "Model: " << model << "\n";
    std::cout << "Firmware: " << firmware << "\n";
    std::cout << "Hardware: " << hardware << "\n";
    std::cout << "Serialnum: " << serialnum << "\n";
}

void Display::state()
{
    auto [code, name] = getstate();
    std::cout << "Current status: " << std::quoted(name) << " ["
              << (uint32_t)code << "]\n";
}

void Display::samplerate()
{
    auto [normalms, expressms] = getsamplerate();
    std::cout << "Normal scan: " << normalms << "ms\n";
    std::cout << "Express scan: " << expressms << "ms\n";
}

void Display::configuration()
{
    auto config = getconfiguration();

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
        std::cout << "\t> cost per sample: " << mode.uscostpersample << " us\n";
        std::cout << "\t> max sample rate: " << mode.maxsamplerate << " sps\n";
        std::cout << "\t> max distance: " << mode.maxdistance << "m\n";
        std::cout << "\t> answer cmd type: " << std::hex << std::showbase
                  << mode.answercmdtype << std::noshowbase << std::dec << "\n";
        std::cout << "\n";
    });
}

void Display::normalscanning(std::shared_ptr<ScanningIf> scan)
{
    system("clear");
    std::cout << "Normal 360 scan started @ " << gettimestr();
    std::cout << "\e[?25l\n"; // hide cursor
    statustest(scan->observer);
    runnormalscan();
    getchar();
    stopnormalscan();
    system("clear");
    std::cout << "Normal 360 scan completed";
    std::cout << "\e[?25h\n"; // show cursor
}

void Display::expressscanning(std::shared_ptr<ScanningIf> scan)
{
    system("clear");
    std::cout << "Express 360 " << std::quoted(expressscantype)
              << " scan started @ " << gettimestr();
    std::cout << "\e[?25l\n"; // hide cursor
    statustest(scan->observer);
    runnormalscan();
    getchar();
    stopnormalscan();
    system("clear");
    std::cout << "Express 360 scan completed";
    std::cout << "\e[?25h\n"; // show cursor
}

void Display::exitprogram()
{
    std::cout << "Cleaning and closing\n";
}

void Display::run()
{
    Menu("[Lidar " + modelname + " scanner on " + device + " @ " + baud + "]",
         {{"get info", std::bind(&Display::info, this)},
          {"get status", std::bind(&Display::state, this)},
          {"get sampling time", std::bind(&Display::samplerate, this)},
          {"get configuration", std::bind(&Display::configuration, this)},
          {"normal scanning",
           std::bind(&Display::normalscanning, this, normalscan)},
          {"express scanning [" + expressscantype + "]",
           std::bind(&Display::expressscanning, this, expressscan)},
          {"exit", [this]() { exitprogram(); }}})
        .run();
}
