#include "source/simulation.h"
#include "source/helpers/helper_math.h"
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
                            tsp_int velocity) {
  static tsp_id vehicleId = 1;
  // std::cout << "TSP: lane " << startLane << " " << roadLanes.size()
  //          << std::endl;
  if (startLane >= roadLanes.size() ||
      startPosition >= roadLanes[startLane].pointsCount ||
      roadLanes[startLane].points[startPosition] != 0) {
    return false;
  }

  tsp_vehicle vehicle;
  vehicle.velocity = velocity;
  vehicle.lane = startLane;
  vehicle.position = startPosition;
  vehicle.id = vehicleId;
  vehicles.push_back(vehicle);
  roadLanes[startLane].points[startPosition] = vehicles.back().id;

  // std::cout << "TSP: new vehicles size: " << vehicles.size() << std::endl;

  vehicleId++;
  return true;
}

tsp_id Simulation::addLane(tsp_float spaceLengthM, tsp_float lengthM,
                           tsp_int maxVelocity) {
  tsp_int spacesCount =
      static_cast<tsp_int>(std::round(lengthM / spaceLengthM));
  tsp_id newLaneId = static_cast<tsp_id>(roadLanes.size());
  roadLanes.push_back(
      tsp_road_lane(spacesCount, spaceLengthM, maxVelocity, newLaneId));
  return newLaneId;
}

bool Simulation::setTime(tsp_float newTime) {
  if (newTime <= time) {
    return false;
  }

  tsp_float timeToSet = time;
  for (; timeToSet + timeStep <= newTime; timeToSet += timeStep) {
    for (auto &vehicle : vehicles) {
      tsp_int distance = distanceToTheNextVehicle(vehicle);
      if (distance < vehicle.velocity + maximalAcceleration) {
        vehicle.newVelocity = distance - 1;
      } else if (distance == vehicle.velocity + maximalAcceleration + 1) {
        vehicle.newVelocity = vehicle.velocity + maximalAcceleration;
      } else {
        vehicle.newVelocity = vehicle.velocity;
      }

      if (vehicle.newVelocity > roadLanes[vehicle.lane].maxVelocity) {
        vehicle.newVelocity = roadLanes[vehicle.lane].maxVelocity;
      }

      auto randomValue = HelperMath::getRandom();
      // std::cout << "TSP: rand " << randomValue << " "
      //          << velocityDecreaseProbability << " " << vehicle->velocity
      //          << std::endl;
      if (randomValue <= velocityDecreaseProbability) {
        if (vehicle.newVelocity > randomDeceleration)
          vehicle.newVelocity -= randomDeceleration;
        else if (vehicle.newVelocity > 0)
          vehicle.newVelocity = 0;
      }
    }

    for (auto &vehicle : vehicles) {
      vehicle.velocity = vehicle.newVelocity;
      if (vehicle.velocity > 0) {
        roadLanes[vehicle.lane].points[vehicle.position] = 0;
        vehicle.position += vehicle.velocity;
        vehiclesCoveredDistanceM += static_cast<tsp_float>(vehicle.velocity) *
                                    roadLanes[0].spaceLengthM;
        if (vehicle.position >= roadLanes[0].pointsCount) {
          vehicle.position -= roadLanes[0].pointsCount;
        }
        roadLanes[vehicle.lane].points[vehicle.position] = vehicle.id;
      }
    }
    // std::cout << "TSP velocities: ";
    // for (auto v : vCounts) {
    //  std::cout << v << " ";
    //}
    // std::cout << std::endl;
    // std::cout << "TSP: vehicles " << vehicles.size() << std::endl;
    // std::cout << "TSP: time " << statisticsStartTime +
    // gatheringStatisticsTime
    //          << " " << time + timeTick << std::endl;
  }
  time = timeToSet;
  return true;
}

tsp_simulation_result
Simulation::simulate(tsp_float newVehicleVelocityMps, tsp_float accelerationMps,
                     tsp_float randomDecelerationMps,
                     tsp_float vehicleOccupiedSpaceM, tsp_float spaceLengthM,
                     tsp_float carDensity, tsp_float simulationDurationS) {
  minimalDistance =
      static_cast<tsp_int>(std::ceil(vehicleOccupiedSpaceM / spaceLengthM));
  if (minimalDistance < 1) {
    minimalDistance = 1;
  }
  tsp_int carsToSpawnCount = static_cast<tsp_int>(
      static_cast<tsp_float>(roadLanes[0].pointsCount / minimalDistance) *
      carDensity);
  std::cout << "TSP to spawn " << carsToSpawnCount << std::endl;
  tsp_int carsLeftToSpawn = carsToSpawnCount;
  while (carsLeftToSpawn > 0) {
    tsp_int realSpacesCount = roadLanes[0].pointsCount / minimalDistance;
    tsp_int newPosition =
        static_cast<tsp_int>(std::ceil(static_cast<tsp_float>(realSpacesCount) *
                                       HelperMath::getRandom())) -
        1;
    newPosition *= minimalDistance;
    tsp_int newVelocity =
        static_cast<tsp_int>(newVehicleVelocityMps / spaceLengthM);
    if (addVehicle(0, newPosition, newVelocity)) {
      carsLeftToSpawn--;
    }
  }

  maximalAcceleration =
      static_cast<tsp_int>(std::round(accelerationMps / spaceLengthM));
  randomDeceleration =
      static_cast<tsp_int>(std::round(randomDecelerationMps / spaceLengthM));
  std::cout << "TSP min distance " << minimalDistance << " max acc "
            << maximalAcceleration << " rand dec " << randomDeceleration
            << " max vel " << roadLanes[0].maxVelocity << std::endl;

  setTime(simulationDurationS * second);
  tsp_simulation_result results;
  if (carsToSpawnCount == 0) {
    results.vehiclesPerTime = 0;
  } else {
    results.vehiclesPerTime =
        vehiclesCoveredDistanceM /
        (static_cast<tsp_float>(roadLanes[0].pointsCount) * spaceLengthM) *
        1000.0 / simulationDurationS;
  }
  results.vehiclesDensity =
      static_cast<tsp_float>(carsToSpawnCount * minimalDistance) /
      static_cast<tsp_float>(roadLanes[0].pointsCount);
  std::cout << "TSP: results: " << results.vehiclesPerTime << " "
            << results.vehiclesDensity << std::endl;
  return results;
}

bool Simulation::tspGetPositions(TSP::tsp_vehicle_position *vehiclePositions) {
  std::cout << "TSP: vehicles size " << vehicles.size() << std::endl;
  for (int i = 0; i < vehicles.size(); i++) {
    vehiclePositions[i].lane = vehicles[i].lane;
    vehiclePositions[i].position = vehicles[i].position;
  }
  return true;
}

bool Simulation::setP(tsp_float p) {
  if (p < 0.0 || p > 1.0) {
    return false;
  }
  velocityDecreaseProbability = p;
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
void Simulation::overrideAxleAngle(TSP::tsp_id vehicle, TSP::tsp_float angle) {
  vehicles[vehicle].axleAngle = angle;
}
*/

tsp_int Simulation::distanceToTheNextVehicle(tsp_vehicle &vehicle) {
  for (tsp_int d = 1; d <= vehicle.velocity + maximalAcceleration; d++) {
    if (roadLanes[vehicle.lane]
            .points[(vehicle.position + d + minimalDistance - 1) %
                    roadLanes[vehicle.lane].pointsCount] != 0)
      return d;
  }
  return vehicle.velocity + maximalAcceleration + 1;
}

} // namespace TSP
