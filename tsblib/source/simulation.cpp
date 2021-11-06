#include "source/simulation.h"
#include "source/helpers/helper_math.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

namespace TSP {

bool Simulation::addVehicle(tsp_id startLane, tsp_int startPosition,
                            tsp_int velocity, tsp_float safeTimeHeadwayS,
                            tsp_int safetyGap, bool isAutonomous) {
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
  vehicle.isAutonomous = isAutonomous;
  vehicles.push_back(vehicle);
  roadLanes[startLane]->points[startPosition].vehicle = vehicle.id;
  roadLanes[startLane]->vehiclesCount++;

  return true;
}

void Simulation::addLane(tsp_float lengthM, tsp_int laneCount,
                         tsp_int maxVelocity,
                         tsp_traffic_lights_data &trafficLightsData) {
  tsp_int spacesCount =
      static_cast<tsp_int>(std::round(lengthM / spaceLengthM));
  for (int i = 0; i < laneCount; i++) {
    tsp_id newLaneId = static_cast<tsp_id>(roadLanes.size());
    roadLanes.push_back(
        new tsp_road_lane(spacesCount, spaceLengthM, maxVelocity, newLaneId,
                          (newLaneId == 0 ? &trafficLightsData : nullptr)));
  }
}

bool Simulation::setTime(tsp_float newTimeS) {
  tsp_int newTime = HelperMath::userTimeToSimulationTime(newTimeS);

  if (newTime <= time) {
    return false;
  }

  tsp_int timeToSet = time;
  for (; timeToSet + timeStep <= newTime; timeToSet += timeStep) {
    if (timeToSet % v2vCommunicationTimeInterval == 0) {
      v2vCommunication();
    }
    if (timeToSet % v2iCommunicationTimeInterval == 0) {
      v2iCommunication();
    }
    if (timeToSet % vehiclesStateAndPositionUpdateTimeInterval == 0) {
      updateVehicleStatusAndPosition();
    }
  }
  time = timeToSet;
  return true;
}

void Simulation::v2vCommunication() {
  // TODO remove
  /*
  for (auto &vehicle : vehicles) {
    if (!vehicle.isAutonomous) {
      continue;
    }
    auto &previousVehicle = vehicles[vehicle.previousVehicle - 1];
    if (!previousVehicle.isAutonomous) {
      continue;
    }
    if (previousVehicle.velocity > vehicle.velocity &&
        distanceToTheNextVehicle(previousVehicle) <= maximalAcceleration) {
      previousVehicle.velocity = vehicle.velocity;
    }
  }
  */
}

void Simulation::v2iCommunication() {
  for (auto &vehicle : vehicles) {
    if (!vehicle.isAutonomous) {
      continue;
    }
    tsp_int distanceToNextLight = roadLanes[vehicle.lane]->pointsCount;
    tsp_lane_point *nextLight = nullptr;
    for (auto light : roadLanes[vehicle.lane]->pointsWithTrafficLights) {
      if (vehicle.position == light->position) {
        continue;
      }
      tsp_int d = light->position - vehicle.position;
      if (d < 0) {
        d += roadLanes[vehicle.lane]->pointsCount;
      }
      // TODO make adjustable and use real life value
      if (d * spaceLengthM > 200) {
        continue;
      }
      if (d < distanceToNextLight) {
        nextLight = light;
        distanceToNextLight = d;
      }
    }
    if (nextLight == nullptr) {
      vehicle.greenWaveVelocity = roadLanes[vehicle.lane]->maxVelocity;
      continue;
    }

    tsp_int timeToCrossLightWithOptimalVelocity =
        (distanceToNextLight + vehicle.velocity + maximalAcceleration - 1) /
        (vehicle.velocity + maximalAcceleration);
    tsp_int timeInfluencingLightState =
        timeToCrossLightWithOptimalVelocity %
        ((nextLight->redLightDurationS + nextLight->greenLightDurationS) /
         second);
    bool isLightRedOnArrival = nextLight->isTrafficLightRed;
    tsp_int timeToNextLightState = nextLight->timeToNextState / second;
    while (timeToNextLightState <= timeInfluencingLightState) {
      isLightRedOnArrival = !isLightRedOnArrival;
      timeInfluencingLightState -= timeToNextLightState;
      timeToNextLightState =
          (isLightRedOnArrival ? nextLight->redLightDurationS
                               : nextLight->greenLightDurationS) /
          second;
    }
    if (!isLightRedOnArrival) {
      vehicle.greenWaveVelocity = vehicle.velocity + maximalAcceleration;
      continue;
    }

    tsp_int additionalTimeNeededToCrossGreen =
        nextLight->redLightDurationS / second - timeInfluencingLightState;
    tsp_float recommendedVelocity =
        static_cast<tsp_float>(distanceToNextLight) /
        static_cast<tsp_float>(timeToCrossLightWithOptimalVelocity +
                               additionalTimeNeededToCrossGreen);
    vehicle.greenWaveVelocity =
        static_cast<tsp_int>(std::floor(recommendedVelocity));
  }
}

bool Simulation::setP(tsp_float p) {
  if (p < 0.0 || p > 1.0) {
    return false;
  }
  velocityDecreaseProbability = p;
  return true;
}

bool Simulation::setSpaceLengthM(tsp_float a_spaceLengthM) {
  if (a_spaceLengthM <= 0.0) {
    return false;
  }
  spaceLengthM = a_spaceLengthM;
  return true;
}

tsp_int Simulation::getRoadLanesCount() { return roadLanes.size(); }

tsp_int Simulation::getRoadLanePointsCount() {
  if (roadLanes.empty()) {
    return 0;
  }
  return roadLanes[0]->pointsCount;
}

tsp_int Simulation::getVehiclesCount() { return vehicles.size(); }

void Simulation::getVehicles(tsp_vehicle_state *vehicleState) {
  for (int i = 0; i < vehicles.size(); i++) {
    // TODO return breaking light status
    vehicleState[i].lane = vehicles[i].lane;
    vehicleState[i].position = vehicles[i].position;
    vehicleState[i].velocity = vehicles[i].velocity;
    vehicleState[i].usedSpaces = minimalDistance;
    vehicleState[i].isBreaking = vehicles[i].isBreaking;
    vehicleState[i].isAutonomous = vehicles[i].isAutonomous;
  }
}

tsp_int Simulation::getTrafficLightsCount() {
  tsp_int trafficLightsCount = 0;
  for (auto &lane : roadLanes) {
    trafficLightsCount += lane->pointsWithTrafficLights.size();
  }
  return trafficLightsCount;
}

void Simulation::getTrafficLights(tsp_traffic_light_state *trafficLightState) {
  tsp_id index = 0;
  for (auto &lane : roadLanes) {
    for (auto point : lane->pointsWithTrafficLights) {
      trafficLightState[index].lane = lane->id;
      trafficLightState[index].position = point->position;
      trafficLightState[index].isTrafficLightRed = point->isTrafficLightRed;
      trafficLightState[index].timeToNextState = static_cast<tsp_int>(
          std::ceil(static_cast<tsp_float>(point->timeToNextState) /
                    static_cast<tsp_float>(second)));
      index++;
    }
  }
}

void Simulation::initialize(
    tsp_float newVehicleVelocityMps, tsp_float accelerationMps,
    tsp_float randomDecelerationMps, tsp_float vehicleOccupiedSpaceM,
    tsp_float safetyGapM, tsp_float carDensity, tsp_float safeTimeHeadwayS,
    bool a_allowLaneChanging, tsp_float autonomousCarsPercent) {

  minimalDistance =
      static_cast<tsp_int>(std::ceil(vehicleOccupiedSpaceM / spaceLengthM));
  if (minimalDistance < 1) {
    minimalDistance = 1;
  }
  allowLaneChanging = a_allowLaneChanging;
  tsp_int safetyGap =
      static_cast<tsp_int>(std::ceil(safetyGapM / spaceLengthM));
  for (auto lane : roadLanes) {
    tsp_float realSpacesCount =
        static_cast<tsp_float>(lane->pointsCount / minimalDistance);
    tsp_int carsToSpawnCount =
        static_cast<tsp_int>(realSpacesCount * carDensity);
    tsp_int newVelocity =
        static_cast<tsp_int>(newVehicleVelocityMps / spaceLengthM);
#ifndef NDEBUG
    std::cout << "TSP: vehicles to spawn " << carsToSpawnCount << std::endl;
#endif
    tsp_int autonomousCarsToSpawnCount = static_cast<tsp_int>(std::round(
        static_cast<tsp_float>(carsToSpawnCount) * autonomousCarsPercent));
    tsp_int carsLeftToSpawn = carsToSpawnCount;
    std::vector<tsp_int> positions(carsLeftToSpawn);
    while (carsLeftToSpawn > 0) {
      tsp_int newPosition = HelperMath::getRandomInt(
                                0, static_cast<tsp_int>(realSpacesCount) - 1) *
                            minimalDistance;
      bool isAutonomous = (carsLeftToSpawn <= autonomousCarsToSpawnCount);
      if (addVehicle(lane->id, newPosition, newVelocity, safeTimeHeadwayS,
                     safetyGap, isAutonomous)) {
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
}

tsp_simulation_result Simulation::gatherResults(tsp_float simulationDurationS) {
  setTime(simulationDurationS);
  tsp_simulation_result results;
  if (vehicles.size() == 0) {
    results.vehiclesPerTime = 0;
    results.vehiclesDensity = 0;
    return results;
  }
  tsp_float allRoadsDistance = 0;
  tsp_int allPointsCount = 0;
  for (auto lane : roadLanes) {
    allRoadsDistance +=
        static_cast<tsp_float>(lane->pointsCount) * spaceLengthM;
    allPointsCount += lane->pointsCount;
  }
  results.vehiclesPerTime = vehiclesCoveredDistanceM / allRoadsDistance *
                            1000.0 / simulationDurationS;
  results.vehiclesDensity =
      static_cast<tsp_float>(vehicles.size() * minimalDistance) /
      static_cast<tsp_float>(allPointsCount);
  return results;
}

void Simulation::clear() {
  for (auto lane : roadLanes) {
    delete lane;
  }
  time = 0;
  spaceLengthM = 1.0;
  velocityDecreaseProbability = 0;
  vehiclesCoveredDistanceM = 0;
  roadLanes.clear();
  vehicles.clear();
}

tsp_int Simulation::distanceToTheNextVehicle(tsp_vehicle &vehicle) {
  if (vehicle.nextVehicle == vehicle.id) {
    return roadLanes[vehicle.lane]->pointsCount;
  }
  tsp_int d = vehicles[vehicle.nextVehicle - 1].position - vehicle.position;
  if (d < 0) {
    d += roadLanes[vehicle.lane]->pointsCount;
  }
  return d - minimalDistance;
}

tsp_int Simulation::distanceToTheNearestRedTrafficLight(tsp_vehicle &vehicle) {
  tsp_int d = roadLanes[vehicle.lane]->pointsCount;
  if (roadLanes[0]->pointsWithTrafficLights.empty()) {
    return d;
  }
  for (auto trafficLight : roadLanes[0]->pointsWithTrafficLights) {
    if (trafficLight->isTrafficLightRed &&
        trafficLight->position != vehicle.position) {
      tsp_int newDistance = trafficLight->position - vehicle.position;
      if (newDistance < 0) {
        newDistance += roadLanes[vehicle.lane]->pointsCount;
      }
      if (newDistance < d) {
        d = newDistance;
      }
    }
  }
  return d - 1;
}

} // namespace TSP
