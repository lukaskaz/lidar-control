#include "display.hpp"
#include "lidarfactory.hpp"

#include <boost/program_options.hpp>

#include <csignal>
#include <iomanip>
#include <iostream>

void signalHandler(int signal)
{
    if (signal == SIGINT)
    {
        // unclean exit is blocked
    }
}

int main(int argc, char* argv[])
{
    std::signal(SIGINT, signalHandler);

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()("help,h", "produce help message")(
        "device,d", boost::program_options::value<std::string>(),
        "serial device node")("speed,s",
                              boost::program_options::value<std::string>(),
                              "speed of serial communication")(
        "aseries,a", "use A series lidar")("cseries,c", "use C series lidar");

    boost::program_options::variables_map vm;
    boost::program_options::store(
        boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.contains("help"))
    {
        std::cout << desc;
        return 0;
    }

    const auto& device = vm.contains("device")
                             ? vm.at("device").as<std::string>()
                             : "/dev/ttyUSB0";

    try
    {
        auto lidar = LidarFinder::run(device);
        lidar->watchangle(0, [](const SampleData& data) {
            const auto& [angle, distance] = data;
            uint32_t line{20}, dist{static_cast<uint32_t>(distance)};
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
        lidar->watchangle(180, [](const SampleData& data) {
            const auto& [angle, distance] = data;
            uint32_t line{21}, dist{static_cast<uint32_t>(distance)};
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

        Display(lidar).run();
    }
    catch (const std::exception& ex)
    {
        std::cerr << ex.what() << "\n";
    }
    catch (...)
    {
        std::cerr << "Unknown exception occured, aborting!\n";
    }
    return 0;
}
