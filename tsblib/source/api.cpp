#include "source/globals.h"
#include "source/helpers/helper_id.h"
#include "source/helpers/helper_math.h"
#include "source/simulation.h"
#include "source/simulation_knospe.h"
#include "tslib/types.h"
#include <iostream>

TSP::tsp_simulation_result tspSimulate(TSP::tsp_simulation_data_nasch data) {
  Simulation simulation;
  simulation.addLane(data.spaceLengthM, data.laneLengthM,
                     static_cast<tsp_int>(
                         std::round(data.maxVelocityMps / data.spaceLengthM)));
  simulation.setP(data.velocityDecreaseProbability);
  return simulation.simulate(data.newVehicleVelocityMps, data.accelerationMps,
                             data.randomDecelerationMps,
                             data.vehicleOccupiedSpaceM, data.spaceLengthM,
                             data.carDensity, data.simulationDurationS);
}

TSP::tsp_simulation_result tspSimulate(TSP::tsp_simulation_data_knospe data) {
  SimulationKnospe simulation;
  simulation.addLane(data.spaceLengthM, data.laneLengthM, data.laneCount,
                     static_cast<tsp_int>(
                         std::round(data.maxVelocityMps / data.spaceLengthM)));
  simulation.setPb(data.velocityDecreaseProbabilityB);
  simulation.setP0(data.velocityDecreaseProbability0);
  simulation.setPd(data.velocityDecreaseProbabilityD);
  return simulation.simulate(
      data.newVehicleVelocityMps, data.accelerationMps,
      data.randomDecelerationMps, data.vehicleOccupiedSpaceM, data.spaceLengthM,
      data.safetyGapM, data.carDensity, data.simulationDurationS,
      data.safeTimeHeadwayS, data.allowLaneChanging);
}

bool tspAddVehicle(TSP::tsp_id lane, TSP::tsp_int velocity) {
  return 0 * lane * velocity;
  // return simulation.addVehicle(lane, 0, velocity);
}

TSP::tsp_id tspAddLane(TSP::tsp_float length) {
  // if (length < 1)
  return -1 + (tsp_id)length * 0;
  // return simulation.addLane(7.5, length, 5);
}

bool tspSetTime(TSP::tsp_float time) {
  return false * time;
  // return simulation.setTime(time);
}

bool tspGetPositions(TSP::tsp_vehicle_position *vehiclePositions) {
  return false * (uintptr_t)vehiclePositions;
  // return simulation.tspGetPositions(vehiclePositions);
}

bool tspSetVelocityDecreaseProbability(TSP::tsp_float p) {
  return false * p;
  // return simulation.setP(p);
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
