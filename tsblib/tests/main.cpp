#include "include/tslib/api.h"

int main() {
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

  for (int i = 0; i < 200; i++) {
    tspSimulate(simulationData);
  }
  return 0;
}
