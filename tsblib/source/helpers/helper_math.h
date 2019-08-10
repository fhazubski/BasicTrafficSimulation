#pragma once
#include "source/base.h"
#include <cstdlib>
#include <ctime>
#include <functional>

using namespace TSP;

namespace TSP {

class HelperMath {
public:
  template <typename T> static bool inRange(T value, T smaller, T bigger) {
    return !std::less<T>()(value, smaller) && !std::less<T>()(bigger, value);
  }

  static tsp_float degreeToRadian(const tsp_float value);
  static tsp_float kmphToMps(const tsp_float value);

  static tsp_float turnRadius(const tsp_float axleDistance,
                              const tsp_float axleAngle);
  /*
  static void updatePosition(tsp_vehicle &vehicle,
                             const tsp_float velocityChange,
                             const tsp_float axleAngleChange,
                             const tsp_float timeChange);
                                                         */

  // Returns random number x, where 0 < x <= 1
  static tsp_float getRandom();
  // Returns random integer x, where min <= x <= max
  static tsp_int getRandomInt(tsp_int min, tsp_int max);

  static tsp_float lineToRotation(const tsp_position *const line);

  static tsp_int userTimeToSimulationTime(tsp_float userTime);
  static tsp_int userTimeToSimulationTimeFullS(tsp_float userTime);
};

} // namespace TSP
