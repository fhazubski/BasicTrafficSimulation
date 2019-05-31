#pragma once
#include "source/base.h"
#include <vector>

namespace TSP {

class SimulationKnospe {
public:
  bool addVehicle(tsp_id startLane, tsp_int startPosition, tsp_int velocity,
                  tsp_float safeHeadwayTimeS, tsp_int safetyGap);
  tsp_id addLane(tsp_float spaceLengthM, tsp_float lengthM, tsp_int laneCount,
                 tsp_int maxVelocity);
  bool setTime(tsp_float newTime);
  bool setPb(tsp_float pb);
  bool setP0(tsp_float p0);
  bool setPd(tsp_float pd);
  bool tspGetPositions(tsp_vehicle_position *vehiclePositions);
  tsp_simulation_result
  simulate(tsp_float newVehicleVelocityMps, tsp_float accelerationMps,
           tsp_float randomDecelerationMps, tsp_float vehicleOccupiedSpaceM,
           tsp_float spaceLengthM, TSP::tsp_float safetyGapM,
           tsp_float carDensity, tsp_float simulationDurationS,
           tsp_float safeTimeHeadwayS);

private:
  tsp_int distanceToTheNextVehicle(tsp_vehicle &vehicle);
  tsp_int anticipatedVelocity(tsp_vehicle &vehicle);
  bool isWithinSafeTimeHeadway(tsp_vehicle &vehicle);
  bool isNextVehicleBreaking(tsp_vehicle &vehicle);

  tsp_float time = 0;
  tsp_float velocityDecreaseProbabilityWhenNextIsBreaking = 0;
  tsp_float velocityDecreaseProbabilityWhenStopped = 0;
  tsp_float velocityDecreaseProbability = 0;
  tsp_float vehiclesCoveredDistanceM = 0;
  tsp_int maximalAcceleration;
  tsp_int randomDeceleration;
  tsp_int minimalDistance;
  const tsp_float timeStep = second;

  std::vector<tsp_road_lane> roadLanes;
  std::vector<tsp_vehicle> vehicles;
};

} // namespace TSP