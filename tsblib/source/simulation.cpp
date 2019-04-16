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

tsp_id Simulation::addLane(tsp_int length, tsp_int maxVelocity) {
  roadLanes.push_back(tsp_road_lane(length, maxVelocity));
  return static_cast<tsp_id>(roadLanes.size() - 1);
}

bool Simulation::setTime(tsp_float newTime) {
  if (newTime <= time) {
    return false;
  }

  tsp_int newTimeInt = static_cast<tsp_int>(floor(newTime / 1000.0));
  tsp_int timeDifference = newTimeInt - time;
  // std::cout << "TSP time" << time
  //          << " "  << newTime << " " << timeDifference << std::endl;
  for (int timeTick = 0; timeTick < timeDifference; timeTick++) {
    for (auto &vehicle : vehicles) {
      tsp_int distance = distanceToTheNextVehicle(vehicle);
      if (distance <= vehicle.velocity) {
        vehicle.newVelocity = distance - 1;
      } else if (vehicle.velocity < roadLanes[vehicle.lane].maxVelocity &&
                 distance == vehicle.velocity + 2) {
        vehicle.newVelocity = vehicle.velocity + 1;
      } else {
        vehicle.newVelocity = vehicle.velocity;
      }

      auto randomValue = HelperMath::getRandom();
      // std::cout << "TSP: rand " << randomValue << " "
      //          << velocityDecreaseProbability << " " << vehicle->velocity
      //          << std::endl;
      if (randomValue <= velocityDecreaseProbability) {
        if (vehicle.newVelocity > 0)
          vehicle.newVelocity--;
      }
    }

    for (auto &vehicle : vehicles) {
      vehicle.velocity = vehicle.newVelocity;
      if (vehicle.velocity > 0) {
        roadLanes[vehicle.lane].points[vehicle.position] = 0;
        vehicle.position += vehicle.velocity;
        vehiclesCoveredDistance += vehicle.velocity;
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
  time = newTimeInt;
  return true;
}

tsp_simulation_result Simulation::simulate(tsp_int newVehicleVelocity,
                                           tsp_float carDensity) {
  tsp_int carsToSpawnCount =
      static_cast<tsp_float>(roadLanes[0].pointsCount) * carDensity;
  std::cout << "TSP to spawn " << carsToSpawnCount << std::endl;
  tsp_int carsLeftToSpawn = carsToSpawnCount;
  while (carsLeftToSpawn > 0) {
    tsp_int newPosition =
        std::ceil(static_cast<tsp_float>(roadLanes[0].pointsCount) *
                  HelperMath::getRandom()) -
        1;
    if (addVehicle(0, newPosition, newVehicleVelocity)) {
      carsLeftToSpawn--;
    }
  }
  tsp_float simulationTime = 10000 * 1000 * roadLanes[0].maxVelocity;
  setTime(simulationTime);
  tsp_simulation_result results;
  if (carsToSpawnCount == 0) {
    results.vehiclesPerTime = 0;
  } else {
    results.vehiclesPerTime = static_cast<tsp_float>(vehiclesCoveredDistance) /
                              (simulationTime / 1000.0);
  }
  results.vehiclesDensity = carDensity;
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
  for (tsp_int d = 1; d < vehicle.velocity + 2; d++) {
    if (roadLanes[vehicle.lane].points[(vehicle.position + d) %
                                       roadLanes[vehicle.lane].pointsCount] !=
        0)
      return d;
  }
  return vehicle.velocity + 2;
}

} // namespace TSP
