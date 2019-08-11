#pragma once
#include "source/base.h"
#include "source/simulation.h"

namespace TSP {

class SimulationKnospe : public Simulation {
public:
  void updateVehicleStatusAndPosition() override;
  bool setPb(tsp_float pb);
  bool setP0(tsp_float p0);
  void clear() override;

private:
  tsp_int anticipatedVelocity(tsp_vehicle &vehicle);
  bool isWithinSafeTimeHeadway(tsp_vehicle &vehicle);
  inline bool isNextVehicleBreaking(tsp_vehicle &vehicle);
  bool canChangeLaneRightToLeft(tsp_vehicle &vehicle);
  bool canChangeLaneLeftToRight(tsp_vehicle &vehicle);
  bool getSafetyCriterionOfLaneChanging(tsp_vehicle &vehicle, bool rightToLeft);
  tsp_int distanceToThePredecessorOnAnotherLane(tsp_vehicle &vehicle,
                                                bool predecessorIsOnTheLeft);
  bool isThereEnoughSpaceForLaneChange(tsp_vehicle &vehicle, tsp_int dPred,
                                       bool predecessorIsOnTheLeft);
  tsp_vehicle &predecessorVehicle(tsp_vehicle &vehicle, tsp_int dPred,
                                  bool predecessorIsOnTheLeft);
  tsp_vehicle &getNextVehicle(tsp_vehicle &vehicle);

  tsp_float velocityDecreaseProbabilityWhenNextIsBreaking = 0;
  tsp_float velocityDecreaseProbabilityWhenStopped = 0;
};

} // namespace TSP
