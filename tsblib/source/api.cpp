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

bool tspSetTime(TSP::tsp_float time) {
  if (useKnospeSimulation) {
    return knospeSimulation.setTime(time);
  }
  return naSchSimulation.setTime(time);
}

TSP::tsp_int tspGetRoadLanesCount() {
  if (useKnospeSimulation) {
    return knospeSimulation.getRoadLanesCount();
  }
  return naSchSimulation.getRoadLanesCount();
}

TSP::tsp_int tspGetRoadLanePointsCount() {
  if (useKnospeSimulation) {
    return knospeSimulation.getRoadLanePointsCount();
  }
  return naSchSimulation.getRoadLanePointsCount();
}

TSP::tsp_int tspGetVehiclesCount() {
  if (useKnospeSimulation) {
    return knospeSimulation.getVehiclesCount();
  }
  return naSchSimulation.getVehiclesCount();
}

void tspGetVehicles(TSP::tsp_vehicle_state *vehicleState) {
  if (useKnospeSimulation) {
    knospeSimulation.getVehicles(vehicleState);
    return;
  }
  naSchSimulation.getVehicles(vehicleState);
}

TSP::tsp_int tspGetTrafficLightsCount() {
  if (useKnospeSimulation) {
    return knospeSimulation.getTrafficLightsCount();
  }
  return naSchSimulation.getTrafficLightsCount();
}

void tspGetTrafficLights(TSP::tsp_traffic_light_state *trafficLightState) {
  if (useKnospeSimulation) {
    knospeSimulation.getTrafficLights(trafficLightState);
    return;
  }
  naSchSimulation.getTrafficLights(trafficLightState);
}
