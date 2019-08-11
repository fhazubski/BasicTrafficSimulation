#pragma once
#include "source/base.h"

namespace TSP {

class HelperMath {
public:
  // Returns random number x, where 0 < x <= 1
  static tsp_float getRandom();
  // Returns random integer x, where min <= x <= max
  static tsp_int getRandomInt(tsp_int min, tsp_int max);

  static tsp_int userTimeToSimulationTime(tsp_float userTime);
  static tsp_int userTimeToSimulationTimeFullS(tsp_float userTime);
};

} // namespace TSP
