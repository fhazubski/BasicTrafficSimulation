#include "source/helpers/helper_id.h"
#include <iostream>
#include <limits>

namespace TSP {

tsp_id HelperId::lastId = 0;

tsp_id HelperId::next() {
  if (std::numeric_limits<tsp_id>::max() == lastId) {
    // TODO reuse not used ids
    std::cerr << "Exceeded limit of created vehicles!" << std::endl;
    return 0;
  }
  return ++lastId;
}

} // namespace TSP