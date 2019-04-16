#pragma once
#include "source/base.h"
#include <vector>

namespace TSP {

class Simulation {
public:
  bool addVehicle(tsp_id startLane, tsp_int startPosition, tsp_int velocity);
  tsp_id addLane(tsp_int length, tsp_int maxVelocity);
  bool setTime(tsp_float newTime);
  bool setP(tsp_float p);
  bool tspGetPositions(TSP::tsp_vehicle_position *vehiclePositions);
  tsp_simulation_result simulate(tsp_int newVehicleVelocity,
                                 tsp_float carDensity);

private:
  tsp_int distanceToTheNextVehicle(tsp_vehicle &vehicle);

  tsp_int time = 0;
  tsp_float velocityDecreaseProbability = 0;
  tsp_float vehiclesCoveredDistance = 0;

  std::vector<tsp_road_lane> roadLanes;
  std::vector<tsp_vehicle> vehicles;
};

} // namespace TSP
