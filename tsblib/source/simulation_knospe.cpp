#include "source/simulation_knospe.h"
#include "source/helpers/helper_math.h"
#include <algorithm>
#include <cassert>
#include <iostream>

namespace TSP {

void SimulationKnospe::updateVehicleStatusAndPosition() {
  for (auto &vehicle : vehicles) {

    // Step 0 - determination of randomization parameter
    bool isSafeTimeHeadway = isWithinSafeTimeHeadway(vehicle);
    bool isNextBreaking = isNextVehicleBreaking(vehicle);
    tsp_float decelerationProbability;
    if (isNextBreaking && !isSafeTimeHeadway) {
      decelerationProbability = velocityDecreaseProbabilityWhenNextIsBreaking;
    } else if (vehicle.velocity == 0) {
      decelerationProbability = velocityDecreaseProbabilityWhenStopped;
    } else {
      decelerationProbability = velocityDecreaseProbability;
    }
    vehicle.newIsBreaking = false;

    // Step 1 - acceleration
    if ((!isNextBreaking && !vehicle.isBreaking) || isSafeTimeHeadway) {
      vehicle.newVelocity = std::min(vehicle.velocity + maximalAcceleration,
                                     roadLanes[vehicle.lane]->maxVelocity);
    } else {
      vehicle.newVelocity = vehicle.velocity;
    }
    // Step 2 - breaking
    tsp_int distance = distanceToTheNextVehicle(vehicle);
    assert(distance >= 0);
    tsp_int effectiveDistance =
        distance + std::max(anticipatedVelocity(vehicle) - vehicle.safetyGap,
                            static_cast<tsp_int>(0));

    vehicle.newVelocity = std::min(effectiveDistance, vehicle.newVelocity);

    if (vehicle.newVelocity < vehicle.velocity) {
      vehicle.newIsBreaking = true;
    }

    // Step 3 - randomized breaking
    if (HelperMath::getRandom() <= decelerationProbability) {
      vehicle.newVelocity = std::max(vehicle.newVelocity - randomDeceleration,
                                     static_cast<tsp_int>(0));
      if (isNextBreaking && !isSafeTimeHeadway) {
        vehicle.newIsBreaking = true;
      }
    }
  }

  // Step 4 - movement
  for (auto &vehicle : vehicles) {
    vehicle.velocity = vehicle.newVelocity;
    vehicle.isBreaking = vehicle.newIsBreaking;
    if (vehicle.velocity > 0) {
      roadLanes[vehicle.lane]->points[vehicle.position].vehicle = 0;
      vehicle.position += vehicle.velocity;
      vehiclesCoveredDistanceM += static_cast<tsp_float>(vehicle.velocity) *
                                  roadLanes[vehicle.lane]->spaceLengthM;
      if (vehicle.position >= roadLanes[vehicle.lane]->pointsCount) {
        vehicle.position -= roadLanes[vehicle.lane]->pointsCount;
      }
      roadLanes[vehicle.lane]->points[vehicle.position].vehicle = vehicle.id;
    }
  }

  if (allowLaneChanging) {
    // Lane changing
    for (auto &vehicle : vehicles) {
      if (canChangeLaneRightToLeft(vehicle)) {
        vehicle.newLane = vehicle.lane + 1;
      } else if (canChangeLaneLeftToRight(vehicle)) {
        vehicle.newLane = vehicle.lane - 1;
      }
    }

    // Perform lane changes
    for (auto &vehicle : vehicles) {
      if (vehicle.newLane != vehicle.lane) {
        roadLanes[vehicle.lane]->points[vehicle.position].vehicle = 0;
        roadLanes[vehicle.lane]->vehiclesCount--;
        vehicle.lane = vehicle.newLane;
        roadLanes[vehicle.lane]->points[vehicle.position].vehicle = vehicle.id;
        roadLanes[vehicle.lane]->vehiclesCount++;

        vehicles[vehicle.previousVehicle - 1].nextVehicle = vehicle.nextVehicle;
        vehicles[vehicle.nextVehicle - 1].previousVehicle =
            vehicle.previousVehicle;

        auto &nextVehicle = getNextVehicle(vehicle);
        if (vehicle.position == nextVehicle.position) {
          vehicle.nextVehicle = vehicle.id;
          vehicle.previousVehicle = vehicle.id;
          continue;
        }

        auto &previousVehicle = vehicles[nextVehicle.previousVehicle - 1];
        previousVehicle.nextVehicle = vehicle.id;
        vehicle.previousVehicle = previousVehicle.id;
        vehicle.nextVehicle = nextVehicle.id;
        nextVehicle.previousVehicle = vehicle.id;
      }
    }
  }
}

void SimulationKnospe::clear() {
  Simulation::clear();
  velocityDecreaseProbabilityWhenNextIsBreaking = 0;
  velocityDecreaseProbabilityWhenStopped = 0;
}

bool SimulationKnospe::setPb(tsp_float pb) {
  if (pb < 0.0 || pb > 1.0) {
    return false;
  }
  velocityDecreaseProbabilityWhenNextIsBreaking = pb;
  return true;
}

bool SimulationKnospe::setP0(tsp_float p0) {
  if (p0 < 0.0 || p0 > 1.0) {
    return false;
  }
  velocityDecreaseProbabilityWhenStopped = p0;
  return true;
}

tsp_int SimulationKnospe::anticipatedVelocity(tsp_vehicle &vehicle) {
  auto &nextVehicle = vehicles[vehicle.nextVehicle - 1];
  return std::min(distanceToTheNextVehicle(nextVehicle), nextVehicle.velocity);
}

bool SimulationKnospe::isWithinSafeTimeHeadway(tsp_vehicle &vehicle) {
  auto gap = distanceToTheNextVehicle(vehicle);
  tsp_float timeHeadwayS =
      static_cast<tsp_float>(gap) / static_cast<tsp_float>(vehicle.velocity);
  return (timeHeadwayS >= std::min(static_cast<tsp_float>(vehicle.velocity),
                                   vehicle.safeTimeHeadwayS));
}

bool SimulationKnospe::isNextVehicleBreaking(tsp_vehicle &vehicle) {
  return vehicles[vehicle.nextVehicle - 1].isBreaking;
}

bool SimulationKnospe::canChangeLaneRightToLeft(tsp_vehicle &vehicle) {
  if (vehicle.lane == roadLanes.size() - 1) {
    return false;
  }
  bool incentiveCriterion =
      !vehicle.isBreaking &&
      (vehicle.velocity > distanceToTheNextVehicle(vehicle));
  return incentiveCriterion && getSafetyCriterionOfLaneChanging(vehicle, true);
}

bool SimulationKnospe::canChangeLaneLeftToRight(tsp_vehicle &vehicle) {
  if (vehicle.lane == 0) {
    return false;
  }
  if (!getSafetyCriterionOfLaneChanging(vehicle, false)) {
    return false;
  }
  tsp_int dPred = distanceToThePredecessorOnAnotherLane(vehicle, false);
  tsp_float headwayPred =
      static_cast<tsp_float>(dPred) / static_cast<tsp_float>(vehicle.velocity);
  tsp_float headway =
      static_cast<tsp_float>(distanceToTheNextVehicle(vehicle)) /
      static_cast<tsp_float>(vehicle.velocity);
  auto &predVehicle = predecessorVehicle(vehicle, dPred, false);
  bool incentiveCriterion =
      !vehicle.isBreaking && (headwayPred > 3.0) &&
      ((headway > 6.0) ||
       (vehicle.velocity > distanceToTheNextVehicle(predVehicle)));
  return incentiveCriterion;
}

bool SimulationKnospe::getSafetyCriterionOfLaneChanging(tsp_vehicle &vehicle,
                                                        bool rightToLeft) {
  tsp_int destinationLane = vehicle.lane + (rightToLeft ? 1 : -1);
  if (roadLanes[destinationLane]->vehiclesCount == 0) {
    return true;
  }
  tsp_int dPred = distanceToThePredecessorOnAnotherLane(vehicle, rightToLeft);
  if (!isThereEnoughSpaceForLaneChange(vehicle, dPred, rightToLeft)) {
    return false;
  }

  auto &predVehicle = predecessorVehicle(vehicle, dPred, rightToLeft);
  auto &succVehicle = vehicles[predVehicle.previousVehicle - 1];
  tsp_int dSucc =
      distanceToTheNextVehicle(succVehicle) - dPred - minimalDistance;
  tsp_int anticipatedVelocityPred =
      std::min(distanceToTheNextVehicle(predVehicle), predVehicle.velocity);
  tsp_int effectiveDistancePred =
      dPred + std::max(anticipatedVelocityPred - vehicle.safetyGap,
                       static_cast<tsp_int>(0));
  bool safetyCriterion = (effectiveDistancePred >= vehicle.velocity) &&
                         (dSucc >= succVehicle.velocity);
  return safetyCriterion;
}

tsp_int SimulationKnospe::distanceToThePredecessorOnAnotherLane(
    tsp_vehicle &vehicle, bool predecessorIsOnTheLeft) {
  tsp_int destinationLane = vehicle.lane + (predecessorIsOnTheLeft ? 1 : -1);
  tsp_int dPred = 0;
  // TODO optimize
  while (true) {
    if (roadLanes[destinationLane]->points[vehicle.position + dPred].vehicle !=
        0) {
      if (dPred < 0) {
        dPred += roadLanes[destinationLane]->pointsCount;
      }
      return dPred - minimalDistance;
    }
    dPred++;
    assert(dPred != 0);
    if (vehicle.position + dPred >= roadLanes[destinationLane]->pointsCount) {
      dPred -= roadLanes[destinationLane]->pointsCount;
    }
  }
}

bool SimulationKnospe::isThereEnoughSpaceForLaneChange(
    tsp_vehicle &vehicle, tsp_int dPred, bool predecessorIsOnTheLeft) {
  if (dPred < 0) {
    return false;
  }
  auto &predVehicle =
      predecessorVehicle(vehicle, dPred, predecessorIsOnTheLeft);
  tsp_int dSucc =
      distanceToTheNextVehicle(vehicles[predVehicle.previousVehicle - 1]) -
      dPred - minimalDistance;
  return (dSucc >= 0);
}

tsp_vehicle &SimulationKnospe::predecessorVehicle(tsp_vehicle &vehicle,
                                                  tsp_int dPred,
                                                  bool predecessorIsOnTheLeft) {
  tsp_int destinationLane = vehicle.lane + (predecessorIsOnTheLeft ? 1 : -1);
  tsp_int predVehicleOffset = dPred + minimalDistance;
  if (vehicle.position + predVehicleOffset >=
      roadLanes[destinationLane]->pointsCount) {
    predVehicleOffset -= roadLanes[destinationLane]->pointsCount;
  }
  return vehicles[roadLanes[destinationLane]
                      ->points[vehicle.position + predVehicleOffset]
                      .vehicle -
                  1];
}

tsp_vehicle &SimulationKnospe::getNextVehicle(tsp_vehicle &vehicle) {
  if (roadLanes[vehicle.lane]->vehiclesCount == 1) {
    return vehicle;
  }
  // TODO optimize
  tsp_int offset = 1;
  while (true) {
    if (vehicle.position + offset >= roadLanes[vehicle.lane]->pointsCount) {
      offset -= roadLanes[vehicle.lane]->pointsCount;
    }
    if (roadLanes[vehicle.lane]->points[vehicle.position + offset].vehicle !=
        0) {
      return vehicles
          [roadLanes[vehicle.lane]->points[vehicle.position + offset].vehicle -
           1];
    }
    offset++;
    assert(offset != 0);
  }
}

} // namespace TSP
