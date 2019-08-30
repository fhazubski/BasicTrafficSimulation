#include "source/simulation_nasch.h"
#include "source/helpers/helper_math.h"
#include <algorithm>
#include <cassert>
#include <iostream>

namespace TSP {

void SimulationNaSch::updateVehicleStatusAndPosition() {
  for (auto &roadLane : roadLanes) {
    for (auto point : roadLane->pointsWithTrafficLights) {
      point->timeToNextState -= second;
      if (point->timeToNextState <= 0) {
        point->isTrafficLightRed = !point->isTrafficLightRed;
        point->timeToNextState +=
            (point->isTrafficLightRed ? point->redLightDurationS
                                      : point->greenLightDurationS);
      }
    }
  }

  for (auto &vehicle : vehicles) {
    vehicle.newVelocity = getNewVelocity(vehicle);

    auto randomValue = HelperMath::getRandom();
    if ((!vehicle.isAutonomous) &&
        (randomValue <= velocityDecreaseProbability)) {
      if (vehicle.newVelocity > randomDeceleration)
        vehicle.newVelocity -= randomDeceleration;
      else if (vehicle.newVelocity > 0)
        vehicle.newVelocity = 0;
    }
  }

  for (auto &vehicle : vehicles) {
    assert(vehicle.newVelocity <= roadLanes[vehicle.lane]->maxVelocity);
    assert(vehicle.newVelocity >= 0);
    vehicle.velocity = vehicle.newVelocity;
    if (vehicle.velocity > 0) {
      roadLanes[vehicle.lane]->points[vehicle.position].vehicle = 0;
      vehicle.position += vehicle.velocity;
      vehiclesCoveredDistanceM +=
          static_cast<tsp_float>(vehicle.velocity) * roadLanes[0]->spaceLengthM;
      if (vehicle.position >= roadLanes[0]->pointsCount) {
        vehicle.position -= roadLanes[0]->pointsCount;
      }
      roadLanes[vehicle.lane]->points[vehicle.position].vehicle = vehicle.id;
    }
  }
}

tsp_int SimulationNaSch::getNewVelocity(tsp_vehicle &vehicle) {
  tsp_int maxVelocity = std::min(vehicle.velocity + maximalAcceleration,
                                 roadLanes[vehicle.lane]->maxVelocity);
  tsp_int distance = distanceToTheNextVehicle(vehicle);

  tsp_int newVelocity = std::min(maxVelocity, distance);

  distance = distanceToTheNearestRedTrafficLight(vehicle);

  if (distance < newVelocity) {
    newVelocity = distance;
  }

  if (vehicle.isAutonomous) {
    return std::min(newVelocity, vehicle.greenWaveVelocity);
  }
  return newVelocity;

  // If the next vehicle moving more slowly, adjust velocity to prevent sudden
  // velocity changes
  // return std::min(getRecommendedVelocity(vehicle), maxVelocity);
}

tsp_int SimulationNaSch::getRecommendedVelocity(tsp_vehicle &vehicle) {
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
