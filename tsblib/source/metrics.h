#pragma once
#include "tslib/types.h"
#include <limits>

namespace TSP {

// editable variables
constexpr tsp_float meter = 1000;
constexpr tsp_float milisecond = 1;

// other variables, dependending on editables
constexpr tsp_float kilometer = 1000 * meter;

constexpr tsp_float second = 1000 * milisecond;
constexpr tsp_float minute = 60 * second;
constexpr tsp_float hour = 60 * minute;
constexpr tsp_float day = 24 * hour;
/*
constexpr tsp_int maxLength = std::numeric_limits<tsp_int>::max();
constexpr tsp_int maxTime = std::numeric_limits<tsp_int>::max();

// ensure that simulation will support required constraints
constexpr tsp_int requiredLength = 10 * kilometer;
constexpr tsp_int requiredTimespan = 1 * day;

static_assert(maxLength > requiredLength, "Required length is not reachable");
static_assert(maxTime > requiredTimespan, "Required timespan is not reachable");
*/
} // namespace TSP
