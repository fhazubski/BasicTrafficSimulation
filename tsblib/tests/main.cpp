#include "include/tslib/api.h"
#include <iostream>

int main() {
  int repeats = 20;
#ifndef NDEBUG
  repeats = 2;
#endif

  TSP::tsp_traffic_lights_data trafficLightsData;
  trafficLightsData.useRandomizedInputs = true;
  trafficLightsData.trafficLightsPercent = 0.02;
  trafficLightsData.minimalGreenLightDurationS = 60;
  trafficLightsData.maximalGreenLightDurationS = 120;
  trafficLightsData.minimalRedLightDurationS = 60;
  trafficLightsData.maximalRedLightDurationS = 120;

  TSP::tsp_traffic_lights_data trafficLightsDataNotRandomized;
  trafficLightsDataNotRandomized.useRandomizedInputs = false;
  trafficLightsDataNotRandomized.trafficLightsCount = 3;
  trafficLightsDataNotRandomized.spacingPercent = 0.5;
  trafficLightsDataNotRandomized.optimalSpeedPercentOfMaxSpeed = 0.8;
  trafficLightsDataNotRandomized.redLightDurationPercent = 1.0 / 3.0;

  for (int i = 0; i < repeats; i++) {
    TSP::tsp_simulation_data_nasch simulationDataNaSch;
    simulationDataNaSch.maxVelocityMps = 37.5;
    simulationDataNaSch.newVehicleVelocityMps = 37.5;
    simulationDataNaSch.accelerationMps = 7.5;
    simulationDataNaSch.randomDecelerationMps = 7.5;
    simulationDataNaSch.velocityDecreaseProbability = 0.3;
    simulationDataNaSch.vehicleOccupiedSpaceM = 7.5;
    simulationDataNaSch.spaceLengthM = 7.5;
    simulationDataNaSch.laneLengthM = 1000;
    simulationDataNaSch.carDensity = 0.33;
    simulationDataNaSch.autonomousCarsPercent = 0.0;
    simulationDataNaSch.trafficLightsData = trafficLightsData;

    for (int i = 0; i < 20; i++) {
      tspInitializeSimulation(simulationDataNaSch);
      tspGetRoadLanesCount();
      tspGetRoadLanePointsCount();
      tspGetTrafficLightsCount();
      tspGetVehiclesCount();
      tspGatherResults(200);
    }

    simulationDataNaSch.autonomousCarsPercent = 0.5;
    for (int i = 0; i < 20; i++) {
      tspInitializeSimulation(simulationDataNaSch);
      tspGatherResults(200);
    }

    simulationDataNaSch.autonomousCarsPercent = 1.0;
    simulationDataNaSch.trafficLightsData = trafficLightsDataNotRandomized;
    for (int i = 0; i < 20; i++) {
      tspInitializeSimulation(simulationDataNaSch);
      tspGatherResults(200);
    }

    TSP::tsp_simulation_data_knospe simulationData;
    simulationData.maxVelocityMps = 30;
    simulationData.newVehicleVelocityMps = 30;
    simulationData.accelerationMps = 1.5;
    simulationData.randomDecelerationMps = 1.5;
    simulationData.velocityDecreaseProbabilityB = 0.94;
    simulationData.velocityDecreaseProbability0 = 0.5;
    simulationData.velocityDecreaseProbabilityD = 0.1;
    simulationData.safeTimeHeadwayS = 6;
    simulationData.vehicleOccupiedSpaceM = 4.5;
    simulationData.spaceLengthM = 1.5;
    simulationData.safetyGapM = 10.5;
    simulationData.laneCount = 2;
    simulationData.laneLengthM = 1000;
    simulationData.carDensity = 0.3;
    simulationData.allowLaneChanging = true;

    for (int i = 0; i < 20; i++) {
      tspInitializeSimulation(simulationData);
      tspGatherResults(500);
    }

    simulationData.carDensity = 0.015;
    tspInitializeSimulation(simulationData);
    tspGatherResults(1000);
    simulationData.carDensity = 0.010;
    tspInitializeSimulation(simulationData);
    tspGatherResults(1000);
    simulationData.carDensity = 0.005;
    tspInitializeSimulation(simulationData);
    tspGatherResults(1000);
  }
  return 0;
}
