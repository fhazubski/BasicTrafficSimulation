#pragma once
#include "source/base.h"
#include "source/simulation.h"

namespace TSP {

class SimulationNaSch : public Simulation {
public:
  void updateVehicleStatusAndPosition() override;

private:
  tsp_int getNewVelocity(tsp_vehicle &vehicle);
  tsp_int getRecommendedVelocity(tsp_vehicle &vehicle);
};

} // namespace TSP
