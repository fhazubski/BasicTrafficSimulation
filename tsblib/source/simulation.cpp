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
bool Simulation::addVehicle(tsp_id startLane, tsp_int velocity) {
  static tsp_id vehicleId = 1;
  // std::cout << "TSP: lane " << startLane << " " << roadLanes.size()
  //          << std::endl;
  if (startLane >= roadLanes.size() || roadLanes[startLane].points[0] != 0) {
    return false;
  }

  tsp_vehicle vehicle;
  vehicle.velocity = velocity;
  vehicle.lane = startLane;
  vehicle.position = 0;
  vehicle.id = vehicleId;
  vehicles.push_back(vehicle);
  roadLanes[startLane].points[0] = vehicles.back().id;

  std::cout << "TSP: new vehicles size: " << vehicles.size() << std::endl;

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
  for (int timeTick = 0; timeTick < timeDifference; timeTick++) {
    auto vehicle = std::begin(vehicles);
    while (vehicle != std::end(vehicles)) {
      tsp_int distance =
          distanceToTheNextVehicle(*vehicle, vehicle->velocity + 2);
      if (distance <= vehicle->velocity) {
        vehicle->velocity = distance - 1;
      } else if (vehicle->velocity < roadLanes[vehicle->lane].maxVelocity &&
                 distance == vehicle->velocity + 2) {
        vehicle->velocity++;
      }

      if (HelperMath::getRandom() <= velocityDecreaseProbability) {
        if (vehicle->velocity > 0)
          vehicle->velocity--;
      }

      roadLanes[vehicle->lane].points[vehicle->position] = 0;
      vehicle->position += vehicle->velocity;
      if (vehicle->position >= roadLanes[vehicle->lane].pointsCount) {
        vehicle = vehicles.erase(vehicle);
        if (!firstVehicleLeft) {
          statisticsStartTime = time + timeTick;
          gatheringStatisticsTime = 100000 * roadLanes[0].maxVelocity;
          gatheringStatisticsTimeLeft = gatheringStatisticsTime;
          density = 0;
          vehiclesCountStartTime = vehicles.size();
          firstVehicleLeft = true;
        }
        vehiclesLeftCount++;
        continue;
      }

      gatheringStatisticsTimeLeft--;
      density += static_cast<tsp_float>(vehicles.size()) /
                 static_cast<tsp_float>(gatheringStatisticsTime);
      if (gatheringStatisticsTimeLeft == 0) {
        vehiclesPassed = vehiclesLeftCount;
        vehiclesPerTime = static_cast<tsp_float>(vehiclesPassed) /
                          static_cast<tsp_float>(gatheringStatisticsTime);
        std::cout << "TSP results: " << vehiclesPassed << " "
                  << gatheringStatisticsTime << " " << vehiclesPerTime
                  << std::endl;
        density /= static_cast<tsp_float>(roadLanes[0].pointsCount);
      }

      roadLanes[vehicle->lane].points[vehicle->position] = vehicle->id;
      vehicle++;
    }
  }
  time = newTimeInt;
  return true;
}

tsp_simulation_result Simulation::simulate(tsp_int vehicleSpawningInterval) {
  tsp_int currentTime = 0;
  while (!firstVehicleLeft || gatheringStatisticsTimeLeft > 0) {
    addVehicle(0, roadLanes[0].maxVelocity);
    currentTime += vehicleSpawningInterval;
    setTime(currentTime);
  }
  tsp_simulation_result results;
  results.vehiclesPerTime = vehiclesPerTime;
  results.vehiclesDensity = density;
  return results;
}

bool Simulation::tspGetPositions(TSP::tsp_vehicle_position *vehiclePositions) {
  // std::cout << "TSP: " << vehicles.size() << std::endl;
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

tsp_int Simulation::distanceToTheNextVehicle(tsp_vehicle vehicle,
                                             const tsp_int maxDistance) {
  tsp_int newMaxDistance = maxDistance;
  if (roadLanes[vehicle.lane].pointsCount - vehicle.position < maxDistance) {
    newMaxDistance = roadLanes[vehicle.lane].pointsCount - vehicle.position;
  }

  for (tsp_int d = 1; d < newMaxDistance; d++) {
    if (roadLanes[vehicle.lane].points[vehicle.position + d] != 0)
      return d;
  }
  return maxDistance;
}

} // namespace TSP
