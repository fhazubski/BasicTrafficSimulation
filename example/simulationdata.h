#ifndef SIMULATIONDATA_H
#define SIMULATIONDATA_H

#include "tslib/types.h"

struct DataNaSch : TSP::tsp_simulation_data_nasch {
  DataNaSch() {
    maxVelocityMps = 37.5;
    newVehicleVelocityMps = 37.5;
    accelerationMps = 7.5;
    randomDecelerationMps = 7.5;
    velocityDecreaseProbability = 0.1;
    vehicleOccupiedSpaceM = 7.5;
    spaceLengthM = 7.5;
    laneLengthM = 1000;
  }
};

struct DataNaSchP05 : DataNaSch {
  DataNaSchP05() { velocityDecreaseProbability = 0.5; }
};

struct DataNaSchLikeKnospe : TSP::tsp_simulation_data_nasch {
  DataNaSchLikeKnospe() {
    maxVelocityMps = 30;
    newVehicleVelocityMps = 30;
    accelerationMps = 1.5;
    randomDecelerationMps = 1.5;
    velocityDecreaseProbability = 0.1;
    vehicleOccupiedSpaceM = 7.5;
    spaceLengthM = 1.5;
    laneLengthM = 1000;
  }
};

struct DataKnospe : TSP::tsp_simulation_data_knospe {
  DataKnospe() {
    maxVelocityMps = 30;
    newVehicleVelocityMps = 30;
    accelerationMps = 1.5;
    randomDecelerationMps = 1.5;
    velocityDecreaseProbabilityB = 0.94;
    velocityDecreaseProbability0 = 0.5;
    velocityDecreaseProbabilityD = 0.1;
    safeTimeHeadwayS = 6;
    vehicleOccupiedSpaceM = 7.5;
    spaceLengthM = 1.5;
    safetyGapM = 10.5;
    laneCount = 2;
    laneLengthM = 1000;
    allowLaneChanging = true;
  }
};

struct DataKnospeWithoutLaneChanging : DataKnospe {
  DataKnospeWithoutLaneChanging() { allowLaneChanging = false; }
};

#endif // SIMULATIONDATA_H
