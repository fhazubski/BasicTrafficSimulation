#include "source/helpers/helper_math.h"
#include <algorithm>
#include <cstdlib>
#include <ctime>

namespace TSP {

// Returns random number x, where 0 < x <= 1
tsp_float HelperMath::getRandom() {
  static bool initialize = true;
  if (initialize) {
    srand(static_cast<unsigned>(time(nullptr)));
    initialize = false;
  }
  return (static_cast<tsp_float>(rand() + 1) /
          static_cast<tsp_float>(RAND_MAX + 1));
}

// Returns random integer x, where min <= x <= max
tsp_int HelperMath::getRandomInt(tsp_int min, tsp_int max) {
  auto result = std::ceil(static_cast<tsp_float>(max - min + 1) * getRandom());
  return static_cast<tsp_int>(result) + min - 1;
}

tsp_int HelperMath::userTimeToSimulationTime(tsp_float userTime) {
  return static_cast<tsp_int>(
      std::round(userTime * static_cast<tsp_float>(second)));
}

tsp_int HelperMath::userTimeToSimulationTimeFullS(tsp_float userTime) {
  return static_cast<tsp_int>(std::round(userTime)) * second;
}

} // namespace TSP
