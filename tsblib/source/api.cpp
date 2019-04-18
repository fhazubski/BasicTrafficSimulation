#include "source/globals.h"
#include "source/helpers/helper_id.h"
#include "source/helpers/helper_math.h"
#include "tslib/types.h"
#include <iostream>

TSP::tsp_simulation_result
tspSimulate(TSP::tsp_float maxVelocityMps, TSP::tsp_float newVehicleVelocityMps,
            TSP::tsp_float velocityChangeMps,
            TSP::tsp_float velocityDecreaseProbability,
            TSP::tsp_float vehicleOccupiedSpaceM, TSP::tsp_float spaceLengthM,
            TSP::tsp_float laneLengthM, TSP::tsp_float carDensity,
            TSP::tsp_float simulationDurationS) {
  Simulation simulation;
  simulation.addLane(spaceLengthM, laneLengthM,
                     std::round(maxVelocityMps / spaceLengthM));
  simulation.setP(velocityDecreaseProbability);
  return simulation.simulate(newVehicleVelocityMps, velocityChangeMps,
                             vehicleOccupiedSpaceM, spaceLengthM, carDensity,
                             simulationDurationS);
}

bool tspAddVehicle(TSP::tsp_id lane, TSP::tsp_int velocity) {
  return simulation.addVehicle(lane, 0, velocity);
}

TSP::tsp_id tspAddLane(TSP::tsp_int length) {
  if (length < 1)
    return -1;
  return simulation.addLane(7.5, length, 5);
}

bool tspSetTime(TSP::tsp_float time) { return simulation.setTime(time); }

bool tspGetPositions(TSP::tsp_vehicle_position *vehiclePositions) {
  return simulation.tspGetPositions(vehiclePositions);
}

bool tspSetVelocityDecreaseProbability(TSP::tsp_float p) {
  return simulation.setP(p);
}

/*
TSP::tsp_vehicle *tspReserveVehicleMemory(TSP::tsp_int vehiclesCount) {
  return simulation.reserveVehicleMemory(vehiclesCount);
}

TSP::tsp_obstacle_line *tspReserveObstacleMemory(TSP::tsp_int obstacleCount) {
  return simulation.reserveObstacleMemory(obstacleCount);
}
bool tspAddVehicle(TSP::tsp_float width, TSP::tsp_float height,
                   TSP::tsp_float axleDistance,
                   TSP::tsp_float frontWheelsDistance, TSP::tsp_float velocity,
                   const TSP::tsp_road *const startRoad,
                   TSP::tsp_int startLane) {
  if (!HelperMath::inRange<tsp_float>(axleDistance, 0.0, height) &&
      axleDistance != 0.0) {
    std::cerr << "Axle value is less than or equal zero or greater than height"
              << std::endl;
    return false;
  }

  velocity = HelperMath::kmphToMps(velocity);

  return simulation.addVehicle(width, height, axleDistance, frontWheelsDistance,
                               velocity, startRoad, startLane);
}

void tspOverrideAxleAngle(TSP::tsp_id vehicle, TSP::tsp_float angle) {
  angle = HelperMath::degreeToRadian(angle);
  simulation.overrideAxleAngle(vehicle, angle);
}

bool tspSetTime(TSP::tsp_float time) { return simulation.setTime(time); }

TSP::tsp_vehicle_position *tspGetPositions() { return nullptr; }
*/
