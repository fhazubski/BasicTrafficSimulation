#pragma once
#include "source/base.h"
#include <vector>

namespace TSP {

class SimulationKnospe {
public:
  ~SimulationKnospe() {
    for (auto lane : roadLanes) {
      delete lane;
    }
  }
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
           tsp_float safeTimeHeadwayS, bool a_allowLaneChanging);

private:
  inline tsp_int distanceToTheNextVehicle(tsp_vehicle &vehicle);
  tsp_int anticipatedVelocity(tsp_vehicle &vehicle);
  bool isWithinSafeTimeHeadway(tsp_vehicle &vehicle);
  inline bool isNextVehicleBreaking(tsp_vehicle &vehicle);
  bool canChangeLaneRightToLeft(tsp_vehicle &vehicle);
  bool canChangeLaneLeftToRight(tsp_vehicle &vehicle);
  bool getSafetyCriterionOfLaneChanging(tsp_vehicle &vehicle, tsp_int dPred,
                                        bool rightToLeft);
  tsp_int distanceToThePredecessorOnAnotherLane(tsp_vehicle &vehicle,
                                                bool rightToLeft);
  tsp_vehicle &predecessorVehicle(tsp_vehicle &vehicle, tsp_int dPred,
                                  bool rightToLeft);
  tsp_vehicle &getNextVehicle(tsp_vehicle &vehicle);

  tsp_float time = 0;
  tsp_float velocityDecreaseProbabilityWhenNextIsBreaking = 0;
  tsp_float velocityDecreaseProbabilityWhenStopped = 0;
  tsp_float velocityDecreaseProbability = 0;
  tsp_float vehiclesCoveredDistanceM = 0;
  tsp_int maximalAcceleration;
  tsp_int randomDeceleration;
  tsp_int minimalDistance;
  const tsp_float timeStep = second;
  bool allowLaneChanging;

  std::vector<tsp_road_lane *> roadLanes;
  std::vector<tsp_vehicle> vehicles;
};

} // namespace TSP
