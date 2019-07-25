#ifndef SIMULATIONDATA_H
#define SIMULATIONDATA_H

#include "tslib/types.h"

struct NoTrafficLights : TSP::tsp_traffic_lights_data {
  NoTrafficLights() {
    useRandomizedInputs = false;
    trafficLightsPercent = 0;
    trafficLightsCount = 0;
  }
};

struct TrafficLightsRandomized : TSP::tsp_traffic_lights_data {
  TrafficLightsRandomized() {
    useRandomizedInputs = true;
    trafficLightsPercent = 0.01;
    minimalRedLightDurationS = 60;
    maximalRedLightDurationS = 60;
    minimalGreenLightDurationS = 60;
    maximalGreenLightDurationS = 60;
  }
};

struct MoreTrafficLightsRandomized : TrafficLightsRandomized {
  MoreTrafficLightsRandomized() { trafficLightsPercent = 0.02; }
};

struct TrafficLightsSingleFullSpeedNotRandomized
    : TSP::tsp_traffic_lights_data {
  TrafficLightsSingleFullSpeedNotRandomized() {
    useRandomizedInputs = false;
    trafficLightsCount = 1;
    spacingPercent = 0.5;
    optimalSpeedPercentOfMaxSpeed = 1.0;
    greenLightDurationS = 60;
    redLightDurationS = 30;
  }
};

struct TrafficLightsThreeFullSpeedNotRandomized
    : TrafficLightsSingleFullSpeedNotRandomized {
  TrafficLightsThreeFullSpeedNotRandomized() { trafficLightsCount = 3; }
};

struct TrafficLightsSingle08SpeedNotRandomized
    : TrafficLightsSingleFullSpeedNotRandomized {
  TrafficLightsSingle08SpeedNotRandomized() {
    optimalSpeedPercentOfMaxSpeed = 0.8;
  }
};

struct TrafficLightsThree08SpeedNotRandomized
    : TrafficLightsSingle08SpeedNotRandomized {
  TrafficLightsThree08SpeedNotRandomized() { trafficLightsCount = 3; }
};

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
    autonomousCarsPercent = 0;
    trafficLightsData = NoTrafficLights();
  }
};

struct DataNaSchP03Auto0 : DataNaSch {
  DataNaSchP03Auto0() { velocityDecreaseProbability = 0.3; }
};

struct DataNaSchP03Auto025 : DataNaSchP03Auto0 {
  DataNaSchP03Auto025() { autonomousCarsPercent = 0.25; }
};

struct DataNaSchP03Auto05 : DataNaSchP03Auto0 {
  DataNaSchP03Auto05() { autonomousCarsPercent = 0.5; }
};

struct DataNaSchP03Auto075 : DataNaSchP03Auto0 {
  DataNaSchP03Auto075() { autonomousCarsPercent = 0.75; }
};

struct DataNaSchP03Auto1 : DataNaSchP03Auto0 {
  DataNaSchP03Auto1() { autonomousCarsPercent = 1; }
};

struct DataNaSchP0225 : DataNaSch {
  DataNaSchP0225() { velocityDecreaseProbability = 0.225; }
};

struct DataNaSchP015 : DataNaSch {
  DataNaSchP015() { velocityDecreaseProbability = 0.15; }
};

struct DataNaSchP0075 : DataNaSch {
  DataNaSchP0075() { velocityDecreaseProbability = 0.075; }
};

struct DataNaSchP05 : DataNaSch {
  DataNaSchP05() { velocityDecreaseProbability = 0.5; }
};

struct DataNaSchP0 : DataNaSch {
  DataNaSchP0() { velocityDecreaseProbability = 0; }
};

struct DataNaSchTrafficLightsP001 : DataNaSch {
  DataNaSchTrafficLightsP001() {
    trafficLightsData = TrafficLightsRandomized();
  }
};

struct DataNaSchTrafficLightsP002 : DataNaSch {
  DataNaSchTrafficLightsP002() {
    trafficLightsData = MoreTrafficLightsRandomized();
  }
};

struct DataNaSchTrafficLightsOneFullSpeed : DataNaSch {
  DataNaSchTrafficLightsOneFullSpeed() {
    trafficLightsData = TrafficLightsSingleFullSpeedNotRandomized();
  }
};

struct DataNaSchTrafficLightsThreeFullSpeed : DataNaSch {
  DataNaSchTrafficLightsThreeFullSpeed() {
    trafficLightsData = TrafficLightsThreeFullSpeedNotRandomized();
  }
};

struct DataNaSchTrafficLightsOne08Speed : DataNaSch {
  DataNaSchTrafficLightsOne08Speed() {
    trafficLightsData = TrafficLightsSingle08SpeedNotRandomized();
  }
};

struct DataNaSchTrafficLightsThree08Speed : DataNaSch {
  DataNaSchTrafficLightsThree08Speed() {
    trafficLightsData = TrafficLightsThree08SpeedNotRandomized();
  }
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
    autonomousCarsPercent = 0;
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

struct DataKnospeWithoutRandomizedBreaking : DataKnospe {
  DataKnospeWithoutRandomizedBreaking() {
    velocityDecreaseProbabilityB = 0;
    velocityDecreaseProbability0 = 0;
    velocityDecreaseProbabilityD = 0;
  }
};

#endif // SIMULATIONDATA_H
