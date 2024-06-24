#include "lidar.hpp"

#include <boost/program_options.hpp>

#include <csignal>
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
        auto lidar = Lidar::detect(device);
        lidar->run();
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
