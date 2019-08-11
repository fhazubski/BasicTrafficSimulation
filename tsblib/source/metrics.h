#pragma once
#include "tslib/types.h"

namespace TSP {

// editable variables
constexpr tsp_float meter = 1000.0;
constexpr tsp_int milisecond = 1;

// other variables, dependending on editables
constexpr tsp_float kilometer = 1000.0 * meter;

constexpr tsp_int second = 1000 * milisecond;
constexpr tsp_int minute = 60 * second;
constexpr tsp_int hour = 60 * minute;
constexpr tsp_int day = 24 * hour;

} // namespace TSP
