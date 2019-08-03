#include "source/simulation.h"
#include "source/helpers/helper_math.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <iterator>

namespace TSP {
/*
tsp_vehicle *Simulation::reserveVehicleMemory(tsp_int count) {
  if (vehicles != nullptr) {
    delete[] vehicles;
  }
  vehicles = new tsp_vehicle[count];
  reservedVehiclesCount = count;
  vehiclesCount = 0;
  nextFree = 0;
  return vehicles;
}

tsp_obstacle_line *Simulation::reserveObstacleMemory(tsp_int count) {
  if (obstacles != nullptr) {
    delete[] obstacles;
  }
  obstacles = new tsp_obstacle_line[count];
  obstacleLinesCount = count;
  return obstacles;
}
*/
bool Simulation::addVehicle(tsp_id startLane, tsp_int startPosition,
                            tsp_int velocity, bool isAutonomous) {
  // std::cout << "TSP: lane " << startLane << " " << roadLanes.size()
  //          << std::endl;
  if (startLane >= roadLanes.size() ||
      startPosition >= roadLanes[startLane].pointsCount ||
      roadLanes[startLane].points[startPosition].vehicle != 0) {
    return false;
  }

  tsp_vehicle vehicle;
  vehicle.velocity = velocity;
  vehicle.lane = startLane;
  vehicle.position = startPosition;
  vehicle.id = static_cast<tsp_id>(vehicles.size() + 1);
  vehicle.isAutonomous = isAutonomous;
  vehicles.push_back(vehicle);
  roadLanes[startLane].points[startPosition].vehicle = vehicles.back().id;

  // std::cout << "TSP: new vehicles size: " << vehicles.size() << std::endl;

  return true;
}

tsp_id Simulation::addLane(tsp_float lengthM, tsp_int maxVelocity,
                           tsp_traffic_lights_data &trafficLightsData) {
  tsp_int spacesCount =
      static_cast<tsp_int>(std::round(lengthM / spaceLengthM));
  tsp_id newLaneId = static_cast<tsp_id>(roadLanes.size());
  roadLanes.push_back(tsp_road_lane(spacesCount, spaceLengthM, maxVelocity,
                                    newLaneId, &trafficLightsData));
  return newLaneId;
}

bool Simulation::setTime(tsp_float newTime) {
  if (newTime <= time) {
    return false;
  }

  tsp_int timeToSet = time;
  for (; timeToSet + timeStep <= newTime; timeToSet += timeStep) {
    if (timeToSet % v2vCommunicationTimeInterval == 0) {
      v2vCommunication();
    }
    if (timeToSet % vehiclesStateAndPositionUpdateTimeInterval == 0) {
      updateVehicleStatusAndPosition();
    }
  }
  time = timeToSet;
  return true;
}

void Simulation::updateVehicleStatusAndPosition() {
  for (auto &vehicle : vehicles) {
    vehicle.newVelocity = getNewVelocity(vehicle);

    auto randomValue = HelperMath::getRandom();
    // std::cout << "TSP: rand " << randomValue << " "
    //          << velocityDecreaseProbability << " " << vehicle->velocity
    //          << std::endl;
    if ((!vehicle.isAutonomous) &&
        (randomValue <= velocityDecreaseProbability)) {
      if (vehicle.newVelocity > randomDeceleration)
        vehicle.newVelocity -= randomDeceleration;
      else if (vehicle.newVelocity > 0)
        vehicle.newVelocity = 0;
    }
  }

  for (auto &vehicle : vehicles) {
    assert(vehicle.newVelocity <= roadLanes[vehicle.lane].maxVelocity);
    assert(vehicle.newVelocity >= 0);
    vehicle.velocity = vehicle.newVelocity;
    if (vehicle.velocity > 0) {
      roadLanes[vehicle.lane].points[vehicle.position].vehicle = 0;
      vehicle.position += vehicle.velocity;
      vehiclesCoveredDistanceM +=
          static_cast<tsp_float>(vehicle.velocity) * roadLanes[0].spaceLengthM;
      if (vehicle.position >= roadLanes[0].pointsCount) {
        vehicle.position -= roadLanes[0].pointsCount;
      }
      roadLanes[vehicle.lane].points[vehicle.position].vehicle = vehicle.id;
    }
  }

  for (auto &roadLane : roadLanes) {
    for (auto point : roadLane.pointsWithTrafficLights) {
      point->timeToNextState -= second;
      if (point->timeToNextState <= 0) {
        point->isTrafficLightRed = !point->isTrafficLightRed;
        point->timeToNextState +=
            (point->isTrafficLightRed ? point->redLightDurationS
                                      : point->greenLightDurationS);
      }
    }
  }
}
void Simulation::v2vCommunication() {
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
}

void Simulation::initialize(tsp_float newVehicleVelocityMps,
                            tsp_float accelerationMps,
                            tsp_float randomDecelerationMps,
                            tsp_float vehicleOccupiedSpaceM,
                            tsp_float carDensity,
                            tsp_float autonomousCarsPercent) {
  minimalDistance =
      static_cast<tsp_int>(std::ceil(vehicleOccupiedSpaceM / spaceLengthM));
  if (minimalDistance < 1) {
    minimalDistance = 1;
  }
  tsp_int carsToSpawnCount = static_cast<tsp_int>(
      static_cast<tsp_float>(roadLanes[0].pointsCount / minimalDistance) *
      carDensity);
  std::cout << "TSP: vehicles to spawn " << carsToSpawnCount << std::endl;
  tsp_int autonomousCarsToSpawnCount = static_cast<tsp_int>(std::round(
      static_cast<tsp_float>(carsToSpawnCount) * autonomousCarsPercent));
  tsp_int carsLeftToSpawn = carsToSpawnCount;
  std::vector<tsp_int> positions(carsLeftToSpawn);
  while (carsLeftToSpawn > 0) {
    tsp_int realSpacesCount = roadLanes[0].pointsCount / minimalDistance;
    tsp_int newPosition =
        static_cast<tsp_int>(std::ceil(static_cast<tsp_float>(realSpacesCount) *
                                       HelperMath::getRandom())) -
        1;
    newPosition *= minimalDistance;
    tsp_int newVelocity =
        static_cast<tsp_int>(newVehicleVelocityMps / spaceLengthM);
    bool isAutonomous = (carsLeftToSpawn <= autonomousCarsToSpawnCount);
    if (addVehicle(0, newPosition, newVelocity, isAutonomous)) {
      carsLeftToSpawn--;
      positions[carsLeftToSpawn] = newPosition;
    }
  }
  std::sort(positions.begin(), positions.end());
  positions.push_back(positions[0]);
  for (size_t i = 0; i < positions.size() - 1; i++) {
    tsp_vehicle &previousVehicle =
        vehicles[roadLanes[0].points[positions[i]].vehicle - 1];
    tsp_vehicle &nextVehicle =
        vehicles[roadLanes[0].points[positions[i + 1]].vehicle - 1];
    previousVehicle.nextVehicle = nextVehicle.id;
    nextVehicle.previousVehicle = previousVehicle.id;
  }

  maximalAcceleration =
      static_cast<tsp_int>(std::round(accelerationMps / spaceLengthM));
  randomDeceleration =
      static_cast<tsp_int>(std::round(randomDecelerationMps / spaceLengthM));
  // std::cout << "TSP min distance " << minimalDistance << " max acc "
  //          << maximalAcceleration << " rand dec " << randomDeceleration
  //          << " max vel " << roadLanes[0].maxVelocity << std::endl;
}

tsp_simulation_result Simulation::gatherResults(tsp_float simulationDurationS) {
  setTime(simulationDurationS * second);
  tsp_simulation_result results;
  if (vehicles.size() == 0) {
    results.vehiclesPerTime = 0;
  } else {
    results.vehiclesPerTime =
        vehiclesCoveredDistanceM /
        (static_cast<tsp_float>(roadLanes[0].pointsCount) * spaceLengthM) *
        1000.0 / simulationDurationS;
  }
  results.vehiclesDensity =
      static_cast<tsp_float>(vehicles.size() * minimalDistance) /
      static_cast<tsp_float>(roadLanes[0].pointsCount);
  // std::cout << "TSP: results: " << results.vehiclesPerTime << " "
  //          << results.vehiclesDensity << std::endl;
  return results;
}

void Simulation::clear() {
  time = 0;
  spaceLengthM = 1.0;
  velocityDecreaseProbability = 0;
  vehiclesCoveredDistanceM = 0;
  roadLanes.clear();
  vehicles.clear();
}
tsp_int Simulation::getRoadLanesCount() { return roadLanes.size(); }

tsp_int Simulation::getRoadLanePointsCount() {
  if (roadLanes.empty()) {
    return 0;
  }
  return roadLanes[0].pointsCount;
}

tsp_int Simulation::getVehiclesCount() { return vehicles.size(); }

void Simulation::getVehicles(tsp_vehicle_state *vehicleState) {
  for (int i = 0; i < vehicles.size(); i++) {
    vehicleState[i].lane = vehicles[i].lane;
    vehicleState[i].position = vehicles[i].position;
    vehicleState[i].velocity = vehicles[i].velocity;
  }
}

tsp_int Simulation::getTrafficLightsCount() {
  tsp_int trafficLightsCount = 0;
  for (auto &lane : roadLanes) {
    trafficLightsCount += lane.pointsWithTrafficLights.size();
  }
  return trafficLightsCount;
}

void Simulation::getTrafficLights(tsp_traffic_light_state *trafficLightState) {
  tsp_id index = 0;
  for (auto &lane : roadLanes) {
    for (auto point : lane.pointsWithTrafficLights) {
      trafficLightState[index].lane = lane.id;
      trafficLightState[index].position = point->position;
      trafficLightState[index].isTrafficLightRed = point->isTrafficLightRed;
      trafficLightState[index].timeToNextState = static_cast<tsp_int>(
          std::ceil(static_cast<tsp_float>(point->timeToNextState) /
                    static_cast<tsp_float>(second)));
      index++;
    }
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
/*
bool Simulation::addVehicle(tsp_float width, tsp_float height,
                            tsp_float axleDistance,
                            tsp_float frontWheelsDistance, tsp_float velocity,
                            const tsp_road *const startRoad,
                            tsp_int startLane) {
  if (nextFree >= reservedVehiclesCount) {
    return false;
  }
  vehicles[nextFree] = tsp_vehicle(startRoad);
  vehicles[nextFree].id = nextFree;
  vehicles[nextFree].width = width;
  vehicles[nextFree].height = height;
  vehicles[nextFree].axleAngle = 0.0;
  vehicles[nextFree].frontWheelsDistance = frontWheelsDistance;
  vehicles[nextFree].axleDistance = axleDistance;
  vehicles[nextFree].velocity = velocity;
  vehicles[nextFree].followedLane = startLane;
  vehicles[nextFree].x = startRoad->lanes[startLane]->points[0].x;
  vehicles[nextFree].y = startRoad->lanes[startLane]->points[0].y;
  vehicles[nextFree].rotation =
HelperMath::lineToRotation(startRoad->lanes[startLane]->points); nextFree++;
  vehiclesCount = nextFree;

  return true;
}
void Simulation::overrideAxleAngle(tsp_id vehicle, tsp_float angle) {
  vehicles[vehicle].axleAngle = angle;
}
*/

tsp_int Simulation::distanceToTheNextVehicle(tsp_vehicle &vehicle) {
  tsp_int d = vehicles[vehicle.nextVehicle - 1].position - vehicle.position;
  if (d < 0) {
    d += roadLanes[vehicle.lane].pointsCount;
  }
  return d - minimalDistance + 1;
}

tsp_int Simulation::distanceToTheNearestRedTrafficLight(tsp_vehicle &vehicle) {
  tsp_int d = roadLanes[vehicle.lane].maxVelocity + minimalDistance;
  if (roadLanes[vehicle.lane].pointsWithTrafficLights.empty()) {
    return d;
  }
  for (auto trafficLight : roadLanes[vehicle.lane].pointsWithTrafficLights) {
    if (trafficLight->isTrafficLightRed &&
        trafficLight->position != vehicle.position) {
      tsp_int newDistance = trafficLight->position - vehicle.position;
      if (newDistance < 0) {
        newDistance += roadLanes[vehicle.lane].pointsCount;
      }
      if (newDistance < d) {
        d = newDistance;
      }
    }
  }
  return d - minimalDistance + 1;
}

tsp_int Simulation::getNewVelocity(tsp_vehicle &vehicle) {
  tsp_int maxVelocity = std::min(vehicle.velocity + maximalAcceleration,
                                 roadLanes[vehicle.lane].maxVelocity);
  tsp_int distance = distanceToTheNextVehicle(vehicle);

  tsp_int newVelocity = std::min(maxVelocity, distance - 1);

  distance = distanceToTheNearestRedTrafficLight(vehicle);

  return std::min(newVelocity, distance - 1);

  // If the next vehicle moving more slowly, adjust velocity to prevent sudden
  // velocity changes
  // return std::min(getRecommendedVelocity(vehicle), maxVelocity);
}

tsp_int Simulation::getRecommendedVelocity(tsp_vehicle &vehicle) {
  // TODO fix this function to do something useful
  tsp_vehicle &nextVehicle = vehicles[vehicle.nextVehicle - 1];
  tsp_int distance = distanceToTheNextVehicle(vehicle);
  tsp_int recommendedVelocity = nextVehicle.velocity;
  tsp_int requiredDistance = recommendedVelocity + 1;
  while (requiredDistance + recommendedVelocity + maximalAcceleration <=
         distance) {
    recommendedVelocity += maximalAcceleration;
    requiredDistance += recommendedVelocity;
  }
  return recommendedVelocity;
}

} // namespace TSP
