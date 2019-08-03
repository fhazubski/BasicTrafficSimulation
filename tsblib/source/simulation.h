#pragma once
#include "source/base.h"
#include <vector>

namespace TSP {

class Simulation {
public:
  bool addVehicle(tsp_id startLane, tsp_int startPosition, tsp_int velocity,
                  bool isAutonomous);
  tsp_id addLane(tsp_float lengthM, tsp_int maxVelocity,
                 tsp_traffic_lights_data &trafficLightsData);
  bool setTime(tsp_float newTime);
  void updateVehicleStatusAndPosition();
  void v2vCommunication();
  bool setP(tsp_float p);
  bool setSpaceLengthM(tsp_float a_spaceLengthM);
  tsp_int getRoadLanesCount();
  tsp_int getRoadLanePointsCount();
  tsp_int getVehiclesCount();
  void getVehicles(tsp_vehicle_state *vehicleState);
  tsp_int getTrafficLightsCount();
  void getTrafficLights(tsp_traffic_light_state *trafficLightState);
  void initialize(tsp_float newVehicleVelocityMps, tsp_float accelerationMps,
                  tsp_float randomDecelerationMps,
                  tsp_float vehicleOccupiedSpaceM, tsp_float carDensity,
                  tsp_float autonomousCarsPercent);
  tsp_simulation_result gatherResults(tsp_float simulationDurationS);
  void clear();

private:
  tsp_int distanceToTheNextVehicle(tsp_vehicle &vehicle);
  tsp_int distanceToTheNearestRedTrafficLight(tsp_vehicle &vehicle);
  tsp_int getNewVelocity(tsp_vehicle &vehicle);
  tsp_int getRecommendedVelocity(tsp_vehicle &vehicle);

  tsp_int time = 0;
  tsp_float spaceLengthM = 1.0;
  tsp_float velocityDecreaseProbability = 0;
  tsp_float vehiclesCoveredDistanceM = 0;
  tsp_int maximalAcceleration;
  tsp_int randomDeceleration;
  tsp_int minimalDistance;
  const tsp_int v2vCommunicationTimeInterval = 10 * milisecond;
  const tsp_int timeStep = v2vCommunicationTimeInterval;
  const tsp_int vehiclesStateAndPositionUpdateTimeInterval = second;

  std::vector<tsp_road_lane> roadLanes;
  std::vector<tsp_vehicle> vehicles;
};

} // namespace TSP
