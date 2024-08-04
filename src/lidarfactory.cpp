#include "lidarfactory.hpp"

#include "lidar.hpp"
#include "scanfactory.hpp"

#include <algorithm>
#include <stdexcept>

std::shared_ptr<LidarIf> LidarFactory::createAseries()
{
    return std::shared_ptr<Lidar>(
        new Lidar(seriesid::amodel, B115200,
                  [](std::shared_ptr<serial> serial) -> Lidar::scansinittype {
                      return {ScanFactory::createNormal(serial),
                              ScanFactory::createExpressLegacy(serial)};
                  }));
}

std::shared_ptr<LidarIf> LidarFactory::createCseries()
{
    return std::shared_ptr<Lidar>(
        new Lidar(seriesid::cmodel, B460800,
                  [](std::shared_ptr<serial> serial) -> Lidar::scansinittype {
                      return {ScanFactory::createNormal(serial),
                              ScanFactory::createExpressDense(serial)};
                  }));
}

std::shared_ptr<LidarIf> LidarFinder::run(const std::string& device)
{
    static const std::vector<std::shared_ptr<LidarIf>> lidars = {
        LidarFactory::createAseries(), LidarFactory::createCseries()};

    if (auto found = std::ranges::find_if(
            lidars, [&device](auto lidar) { return lidar->setup(device); });
        found != lidars.end())
    {
        return *found;
    }
    throw std::runtime_error("Cannot find any lidar");
}
