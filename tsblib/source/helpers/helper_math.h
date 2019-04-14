#pragma once
#include "source/base.h"
#include <functional>
#include <cstdlib>
#include <ctime>

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

  static tsp_float getRandom();

  static tsp_float lineToRotation(const tsp_position *const line);
};

} // namespace TSP
