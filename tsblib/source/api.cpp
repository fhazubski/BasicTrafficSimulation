#include "source/globals.h"
#include "source/helpers/helper_id.h"
#include "source/helpers/helper_math.h"
#include "source/simulation.h"
#include "source/simulation_knospe.h"
#include "tslib/types.h"
#include <iostream>

namespace TSP {
Simulation naSchSimulation;
SimulationKnospe knospeSimulation;
bool useKnospeSimulation;
} // namespace TSP

void tspInitializeSimulation(TSP::tsp_simulation_data_nasch data) {
  naSchSimulation.clear();
  useKnospeSimulation = false;
  naSchSimulation.setSpaceLengthM(data.spaceLengthM);
  naSchSimulation.addLane(
      data.laneLengthM,
      static_cast<tsp_int>(std::round(data.maxVelocityMps / data.spaceLengthM)),
      data.trafficLightsData);
  naSchSimulation.setP(data.velocityDecreaseProbability);
  naSchSimulation.initialize(data.newVehicleVelocityMps, data.accelerationMps,
                             data.randomDecelerationMps,
                             data.vehicleOccupiedSpaceM, data.carDensity,
                             data.autonomousCarsPercent);
}

void tspInitializeSimulation(TSP::tsp_simulation_data_knospe data) {
  knospeSimulation.clear();
  useKnospeSimulation = true;
  naSchSimulation.setSpaceLengthM(data.spaceLengthM);
  knospeSimulation.addLane(data.laneLengthM, data.laneCount,
                           static_cast<tsp_int>(std::round(data.maxVelocityMps /
                                                           data.spaceLengthM)));
  knospeSimulation.setPb(data.velocityDecreaseProbabilityB);
  knospeSimulation.setP0(data.velocityDecreaseProbability0);
  knospeSimulation.setPd(data.velocityDecreaseProbabilityD);
  knospeSimulation.initialize(
      data.newVehicleVelocityMps, data.accelerationMps,
      data.randomDecelerationMps, data.vehicleOccupiedSpaceM, data.safetyGapM,
      data.carDensity, data.safeTimeHeadwayS, data.allowLaneChanging);
}

TSP::tsp_simulation_result
tspGatherResults(TSP::tsp_float simulationDurationS) {
  if (useKnospeSimulation) {
    return knospeSimulation.gatherResults(simulationDurationS);
  }
  return naSchSimulation.gatherResults(simulationDurationS);
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
