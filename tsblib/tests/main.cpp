#include "include/tslib/api.h"

int main() {
#ifdef NDEBUG
  for (int i = 0; i < 100; i++) {
#endif
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
    simulationDataNaSch.simulationDurationS = 200;
    simulationDataNaSch.autonomousCarsPercent = 0.0;

    for (int i = 0; i < 20; i++) {
      tspSimulate(simulationDataNaSch);
    }

    simulationDataNaSch.autonomousCarsPercent = 0.5;
    for (int i = 0; i < 20; i++) {
      tspSimulate(simulationDataNaSch);
    }

    simulationDataNaSch.autonomousCarsPercent = 1.0;
    for (int i = 0; i < 20; i++) {
      tspSimulate(simulationDataNaSch);
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
    simulationData.simulationDurationS = 500;
    simulationData.allowLaneChanging = true;

    for (int i = 0; i < 20; i++) {
      tspSimulate(simulationData);
    }

    simulationData.simulationDurationS = 10000;
    simulationData.carDensity = 0.015;
    tspSimulate(simulationData);
    simulationData.carDensity = 0.010;
    tspSimulate(simulationData);
    simulationData.carDensity = 0.005;
    tspSimulate(simulationData);
#ifdef NDEBUG
  }
#endif
  return 0;
}
