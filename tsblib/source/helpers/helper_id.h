#pragma once
#include "source/base.h"

namespace TSP {

class HelperId {
public:
  static tsp_id next();

private:
  static tsp_id lastId;
};

} // namespace TSP
