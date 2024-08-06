#include "helpers.hpp"

#include <algorithm>
#include <ctime>

std::string gettimestr()
{
    time_t rawtime{0};
    time(&rawtime);
    std::string timestr{asctime(localtime(&rawtime))};
    timestr.erase(std::remove(timestr.begin(), timestr.end(), '\n'),
                  timestr.end());
    return timestr;
}
