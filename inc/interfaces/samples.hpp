#pragma once

#include <cstdint>
#include <functional>
#include <utility>

using SampleData = std::pair<int32_t, double>;
using NotifyFunc = std::function<void(const SampleData&)>;
