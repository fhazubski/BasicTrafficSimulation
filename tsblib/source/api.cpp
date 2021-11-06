#include "tslib/api.h"
#include "source/base.h"
#include "source/simulation_knospe.h"
#include "source/simulation_nasch.h"
#include <cmath>
#include <iostream>

#define CHECK_INIT_AND_ON_FAIL_RETURN(VALUE)                                   \
  if (TSP::simulation == nullptr) {                                            \
    std::cerr << "TSP error: simulation is not initialized!" << std::endl;     \
    return VALUE;                                                              \
  }

namespace TSP {

Simulation *simulation = nullptr;

template <typename T> void allocateSimulation() {
  bool simulationAlreadyAllocated =
      (simulation != nullptr) && (dynamic_cast<T *>(simulation) != nullptr);

  if (simulationAlreadyAllocated) {
    static_cast<T *>(TSP::simulation)->clear();
  } else {
    tspDeinitialize();
    simulation = new T;
  }
}

} // namespace TSP

void tspInitializeSimulation(TSP::tsp_simulation_data_nasch data) {
  TSP::allocateSimulation<TSP::SimulationNaSch>();
  auto naSchSimulation = static_cast<TSP::SimulationNaSch *>(TSP::simulation);
  naSchSimulation->setSpaceLengthM(data.spaceLengthM);
  naSchSimulation->addLane(data.laneLengthM, 1,
                           static_cast<TSP::tsp_int>(std::round(
                               data.maxVelocityMps / data.spaceLengthM)),
                           data.trafficLightsData);
  naSchSimulation->setP(data.velocityDecreaseProbability);
  naSchSimulation->initialize(data.newVehicleVelocityMps, data.accelerationMps,
                              data.randomDecelerationMps,
                              data.vehicleOccupiedSpaceM, 0.0, data.carDensity,
                              0.0, false, data.autonomousCarsPercent);
}

void tspInitializeSimulation(TSP::tsp_simulation_data_knospe data) {
  TSP::allocateSimulation<TSP::SimulationKnospe>();
  auto knospeSimulation = static_cast<TSP::SimulationKnospe *>(TSP::simulation);
  knospeSimulation->setSpaceLengthM(data.spaceLengthM);
  TSP::tsp_traffic_lights_data tlData;
  tlData.enableTrafficLights = false;
  knospeSimulation->addLane(data.laneLengthM, data.laneCount,
                            static_cast<TSP::tsp_int>(std::round(
                                data.maxVelocityMps / data.spaceLengthM)),
                            tlData);
  knospeSimulation->setPb(data.velocityDecreaseProbabilityB);
  knospeSimulation->setP0(data.velocityDecreaseProbability0);
  knospeSimulation->setP(data.velocityDecreaseProbabilityD);
  knospeSimulation->initialize(
      data.newVehicleVelocityMps, data.accelerationMps,
      data.randomDecelerationMps, data.vehicleOccupiedSpaceM, data.safetyGapM,
      data.carDensity, data.safeTimeHeadwayS, data.allowLaneChanging, 0.0);
}

void tspDeinitialize() {
  if (TSP::simulation != nullptr) {
    delete TSP::simulation;
    TSP::simulation = nullptr;
  }
}

TSP::tsp_simulation_result
tspGatherResults(TSP::tsp_float simulationDurationS) {
  CHECK_INIT_AND_ON_FAIL_RETURN({});
  return TSP::simulation->gatherResults(simulationDurationS);
}

bool tspSetTime(TSP::tsp_float timeS) {
  CHECK_INIT_AND_ON_FAIL_RETURN(false);
  return TSP::simulation->setTime(timeS);
}

TSP::tsp_int tspGetRoadLanesCount() {
  CHECK_INIT_AND_ON_FAIL_RETURN(0);
  return TSP::simulation->getRoadLanesCount();
}

TSP::tsp_int tspGetRoadLanePointsCount() {
  CHECK_INIT_AND_ON_FAIL_RETURN(0);
  return TSP::simulation->getRoadLanePointsCount();
}

TSP::tsp_int tspGetVehiclesCount() {
  CHECK_INIT_AND_ON_FAIL_RETURN(0);
  return TSP::simulation->getVehiclesCount();
}

bool tspGetVehicles(TSP::tsp_vehicle_state *vehicleState) {
  CHECK_INIT_AND_ON_FAIL_RETURN(false);
  TSP::simulation->getVehicles(vehicleState);
  return true;
}

TSP::tsp_int tspGetTrafficLightsCount() {
  CHECK_INIT_AND_ON_FAIL_RETURN(0);
  return TSP::simulation->getTrafficLightsCount();
}

bool tspGetTrafficLights(TSP::tsp_traffic_light_state *trafficLightState) {
  CHECK_INIT_AND_ON_FAIL_RETURN(false);
  TSP::simulation->getTrafficLights(trafficLightState);
  return true;
}
