#pragma once
#include "source/base.h"
#include <vector>

namespace TSP {

class Simulation {
public:
  bool addVehicle(tsp_id startLane, tsp_int startPosition, tsp_int velocity,
                  bool isAutonomous);
  tsp_id addLane(tsp_float spaceLengthM, tsp_float lengthM,
                 tsp_int maxVelocity);
  bool setTime(tsp_float newTime);
  void updateVehicleStatusAndPosition();
  void v2vCommunication();
  bool setP(tsp_float p);
  bool tspGetPositions(tsp_vehicle_position *vehiclePositions);
  tsp_simulation_result
  simulate(tsp_float newVehicleVelocityMps, tsp_float accelerationMps,
           tsp_float randomDecelerationMps, tsp_float vehicleOccupiedSpaceM,
           tsp_float spaceLengthM, tsp_float carDensity,
           tsp_float simulationDurationS, tsp_float autonomousCarsPercent);

private:
  tsp_int distanceToTheNextVehicle(tsp_vehicle &vehicle);
  tsp_int getNewVelocity(tsp_vehicle &vehicle);
  tsp_int getRecommendedVelocity(tsp_vehicle &vehicle);

  tsp_int time = 0;
  tsp_float velocityDecreaseProbability = 0;
  tsp_float vehiclesCoveredDistanceM = 0;
  tsp_int maximalAcceleration;
  tsp_int randomDeceleration;
  tsp_int minimalDistance;
  const tsp_int timeStep = v2vCommunicationTimeInterval;
  const tsp_int vehiclesStateAndPositionUpdateTimeInterval = second;
  const tsp_int v2vCommunicationTimeInterval = 10 * milisecond;

  std::vector<tsp_road_lane> roadLanes;
  std::vector<tsp_vehicle> vehicles;
};

} // namespace TSP
