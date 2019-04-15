#pragma once
#include "source/base.h"
#include <vector>

namespace TSP {

class Simulation {
public:
  bool addVehicle(tsp_id startLane, tsp_int velocity);
  tsp_id addLane(tsp_int length, tsp_int maxVelocity);
  bool setTime(tsp_float newTime);
  bool setP(tsp_float p);
  bool tspGetPositions(TSP::tsp_vehicle_position *vehiclePositions);
  tsp_simulation_result simulate(tsp_int vehicleSpawningInterval);

private:
  tsp_int distanceToTheNextVehicle(tsp_vehicle &vehicle,
	  const tsp_int maxDistance);

  tsp_obstacle_line *obstacles = nullptr;
  tsp_int reservedVehiclesCount;
  tsp_int vehiclesCount;
  tsp_int obstacleLinesCount;
  tsp_int nextFree;
  tsp_int time = 0;
  tsp_float velocityDecreaseProbability = 0;

  bool firstVehicleLeft = false;
  tsp_int statisticsStartTime;
  tsp_int vehiclesCountStartTime;
  tsp_int vehiclesLeftCount = 0;
  tsp_int gatheringStatisticsTime;
  tsp_int gatheringStatisticsTimeLeft = 0;
  tsp_float vehiclesPerTime;
  tsp_float density;
  tsp_float vehiclesDensity;
  bool simulationEnded = false;

  std::vector<tsp_road_lane> roadLanes;
  std::vector<tsp_vehicle> vehicles;
};

} // namespace TSP
