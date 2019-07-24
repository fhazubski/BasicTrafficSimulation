#include "source/simulation_knospe.h"
#include "source/helpers/helper_math.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <iterator>
#include <vector>

namespace TSP {

bool SimulationKnospe::addVehicle(tsp_id startLane, tsp_int startPosition,
                                  tsp_int velocity, tsp_float safeTimeHeadwayS,
                                  tsp_int safetyGap) {
  // std::cout << "TSP: lane " << startLane << " " << roadLanes.size()
  //          << std::endl;
  if (startLane >= roadLanes.size() ||
      startPosition >= roadLanes[startLane]->pointsCount ||
      roadLanes[startLane]->points[startPosition].vehicle != 0) {
    return false;
  }

  tsp_vehicle vehicle;
  vehicle.velocity = velocity;
  vehicle.lane = startLane;
  vehicle.newLane = startLane;
  vehicle.position = startPosition;
  vehicle.id = static_cast<tsp_id>(vehicles.size() + 1);
  vehicle.safeTimeHeadwayS = safeTimeHeadwayS;
  vehicle.safetyGap = safetyGap;
  vehicles.push_back(vehicle);
  roadLanes[startLane]->points[startPosition].vehicle = vehicle.id;
  roadLanes[startLane]->vehiclesCount++;

  // std::cout << "TSP: new vehicles size: " << vehicles.size() << std::endl;

  return true;
}

tsp_id SimulationKnospe::addLane(tsp_float spaceLengthM, tsp_float lengthM,
                                 tsp_int laneCount, tsp_int maxVelocity) {
  tsp_int spacesCount =
      static_cast<tsp_int>(std::round(lengthM / spaceLengthM));
  for (int i = 0; i < laneCount; i++) {
    tsp_id newLaneId = static_cast<tsp_id>(roadLanes.size());
    roadLanes.push_back(new tsp_road_lane(spacesCount, spaceLengthM,
                                          maxVelocity, newLaneId, nullptr));
  }
  return static_cast<tsp_id>(roadLanes.size() - 1);
}

bool SimulationKnospe::setTime(tsp_float newTime) {
  if (newTime <= time) {
    return false;
  }

  tsp_float timeToSet = time;
  for (; timeToSet + timeStep <= newTime; timeToSet += timeStep) {
    for (auto &vehicle : vehicles) {

      // Step 0
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

      // Step 1
      if ((!isNextBreaking && !vehicle.isBreaking) || isSafeTimeHeadway) {
        vehicle.newVelocity = std::min(vehicle.velocity + 1,
                                       roadLanes[vehicle.lane]->maxVelocity);
      } else {
        vehicle.newVelocity = vehicle.velocity;
      }
      // Step 2
      tsp_int distance = distanceToTheNextVehicle(vehicle);
      if (distance == minimalDistance * (-1)) {
        distance = roadLanes[vehicle.lane]->maxVelocity * 10;
      } else {
        assert(distance >= 0);
      }
      tsp_int effectiveDistance =
          distance + std::max(anticipatedVelocity(vehicle) - vehicle.safetyGap,
                              static_cast<tsp_int>(0));
      vehicle.newVelocity = std::min(effectiveDistance, vehicle.newVelocity);

      if (vehicle.newVelocity < vehicle.velocity) {
        vehicle.newIsBreaking = true;
      }

      // Step 3
      if (HelperMath::getRandom() <= decelerationProbability) {
        vehicle.newVelocity =
            std::max(vehicle.newVelocity - 1, static_cast<tsp_int>(0));
        if (isNextBreaking && !isSafeTimeHeadway) {
          vehicle.newIsBreaking = true;
        }
      }
    }

    // Step 4
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
          roadLanes[vehicle.lane]->points[vehicle.position].vehicle =
              vehicle.id;
          roadLanes[vehicle.lane]->vehiclesCount++;

          vehicles[vehicle.previousVehicle - 1].nextVehicle =
              vehicle.nextVehicle;
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
  time = timeToSet;
  return true;
}

tsp_simulation_result SimulationKnospe::simulate(
    tsp_float newVehicleVelocityMps, tsp_float accelerationMps,
    tsp_float randomDecelerationMps, tsp_float vehicleOccupiedSpaceM,
    tsp_float spaceLengthM, TSP::tsp_float safetyGapM, tsp_float carDensity,
    tsp_float simulationDurationS, tsp_float safeTimeHeadwayS,
    bool a_allowLaneChanging) {
  minimalDistance =
      static_cast<tsp_int>(std::ceil(vehicleOccupiedSpaceM / spaceLengthM));
  if (minimalDistance < 1) {
    minimalDistance = 1;
  }
  allowLaneChanging = a_allowLaneChanging;
  tsp_int carsToSpawnCount = 0;
  tsp_int safetyGap =
      static_cast<tsp_int>(std::ceil(safetyGapM / spaceLengthM));
  for (auto lane : roadLanes) {
    tsp_float realSpacesCount =
        static_cast<tsp_float>(lane->pointsCount / minimalDistance);
    tsp_int carsLeftToSpawn =
        static_cast<tsp_int>(realSpacesCount * carDensity);
    carsToSpawnCount += carsLeftToSpawn;
    std::cout << "TSP to spawn " << carsLeftToSpawn << std::endl;
    tsp_int newVelocity =
        static_cast<tsp_int>(newVehicleVelocityMps / spaceLengthM);
    std::vector<tsp_int> positions(carsLeftToSpawn);
    while (carsLeftToSpawn > 0) {
      tsp_int newPosition = HelperMath::getRandomInt(
                                0, static_cast<tsp_int>(realSpacesCount) - 1) *
                            minimalDistance;
      if (addVehicle(lane->id, newPosition, newVelocity, safeTimeHeadwayS,
                     safetyGap)) {
        carsLeftToSpawn--;
        positions[carsLeftToSpawn] = newPosition;
      }
    }
    std::sort(positions.begin(), positions.end());
    positions.push_back(positions[0]);
    for (size_t i = 0; i < positions.size() - 1; i++) {
      tsp_vehicle &previousVehicle =
          vehicles[lane->points[positions[i]].vehicle - 1];
      tsp_vehicle &nextVehicle =
          vehicles[lane->points[positions[i + 1]].vehicle - 1];
      previousVehicle.nextVehicle = nextVehicle.id;
      nextVehicle.previousVehicle = previousVehicle.id;
    }
  }

  maximalAcceleration =
      static_cast<tsp_int>(std::round(accelerationMps / spaceLengthM));
  randomDeceleration =
      static_cast<tsp_int>(std::round(randomDecelerationMps / spaceLengthM));
  std::cout << "TSP min distance " << minimalDistance << " max acc "
            << maximalAcceleration << " rand dec " << randomDeceleration
            << " max vel " << roadLanes[0]->maxVelocity << std::endl;

  setTime(simulationDurationS * second);
  tsp_simulation_result results;
  if (carsToSpawnCount == 0) {
    results.vehiclesPerTime = 0;
  } else {
    tsp_float allRoadsDistance = 0;
    for (auto lane : roadLanes) {
      allRoadsDistance +=
          static_cast<tsp_float>(lane->pointsCount) * spaceLengthM;
    }
    results.vehiclesPerTime = vehiclesCoveredDistanceM / allRoadsDistance *
                              1000.0 / simulationDurationS;
  }
  results.vehiclesDensity =
      static_cast<tsp_float>(carsToSpawnCount * minimalDistance) /
      static_cast<tsp_float>(roadLanes[0]->pointsCount * roadLanes.size());
  std::cout << "TSP: results: " << results.vehiclesPerTime << " "
            << results.vehiclesDensity << std::endl;
  return results;
}

bool SimulationKnospe::tspGetPositions(
    TSP::tsp_vehicle_position *vehiclePositions) {
  std::cout << "TSP: vehicles size " << vehicles.size() << std::endl;
  for (int i = 0; i < vehicles.size(); i++) {
    vehiclePositions[i].lane = vehicles[i].lane;
    vehiclePositions[i].position = vehicles[i].position;
  }
  return true;
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

bool SimulationKnospe::setPd(tsp_float pd) {
  if (pd < 0.0 || pd > 1.0) {
    return false;
  }
  velocityDecreaseProbability = pd;
  return true;
}

tsp_int SimulationKnospe::distanceToTheNextVehicle(tsp_vehicle &vehicle) {
  if (vehicles[vehicle.nextVehicle - 1].position < vehicle.position) {
    return vehicles[vehicle.nextVehicle - 1].position +
           roadLanes[vehicle.lane]->pointsCount - vehicle.position -
           minimalDistance;
  }
  return vehicles[vehicle.nextVehicle - 1].position - vehicle.position -
         minimalDistance;
}

tsp_int SimulationKnospe::anticipatedVelocity(tsp_vehicle &vehicle) {
  auto &nextVehicle = vehicles[vehicle.nextVehicle - 1];
  return std::min(distanceToTheNextVehicle(nextVehicle), nextVehicle.velocity);
}

bool SimulationKnospe::isWithinSafeTimeHeadway(tsp_vehicle &vehicle) {
  auto gap = distanceToTheNextVehicle(vehicle);
  tsp_float timeHeadwayS =
      static_cast<tsp_float>(gap) / static_cast<tsp_float>(vehicle.velocity);
  return (timeHeadwayS >= vehicle.safeTimeHeadwayS);
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
  if (!incentiveCriterion) {
    return false;
  }
  if (roadLanes[vehicle.lane + 1]->vehiclesCount == 0) {
    return true;
  }
  tsp_int dPred = distanceToThePredecessorOnAnotherLane(vehicle, true);
  if (!isThereEnoughSpaceForLaneChange(vehicle, dPred, true)) {
    return false;
  }
  return getSafetyCriterionOfLaneChanging(vehicle, dPred, true);
}
bool SimulationKnospe::canChangeLaneLeftToRight(tsp_vehicle &vehicle) {
  if (vehicle.lane == 0) {
    return false;
  }
  if (roadLanes[vehicle.lane - 1]->vehiclesCount == 0) {
    return true;
  }
  tsp_int dPred = distanceToThePredecessorOnAnotherLane(vehicle, false);
  if (!isThereEnoughSpaceForLaneChange(vehicle, dPred, false)) {
    return false;
  }
  tsp_float headwayPred =
      static_cast<tsp_float>(dPred) / static_cast<tsp_float>(vehicle.velocity);
  tsp_float headway =
      static_cast<tsp_float>(distanceToTheNextVehicle(vehicle)) /
      static_cast<tsp_float>(vehicle.velocity);
  bool incentiveCriterion =
      !vehicle.isBreaking && (headwayPred > 3.0) &&
      ((headway > 6.0) ||
       (vehicle.velocity > distanceToTheNextVehicle(vehicle)));
  return incentiveCriterion &&
         getSafetyCriterionOfLaneChanging(vehicle, dPred, false);
}

bool SimulationKnospe::getSafetyCriterionOfLaneChanging(tsp_vehicle &vehicle,
                                                        tsp_int dPred,
                                                        bool rightToLeft) {
  auto &predVehicle = predecessorVehicle(vehicle, dPred, rightToLeft);
  auto &succVehicle = vehicles[predVehicle.previousVehicle - 1];
  tsp_int dSucc = distanceToTheNextVehicle(succVehicle) - dPred - 1;
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
    if (dPred == 0) {
      assert(false);
      return -1;
    }
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
      dPred - 1;
  if (dSucc + 1 < minimalDistance) {
    return false;
  }
  return true;
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
    if (offset == 0) {
      return vehicle;
    }
  }
}

} // namespace TSP
