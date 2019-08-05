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
    minimalRedLightDurationS = 30;
    maximalRedLightDurationS = 60;
    minimalGreenLightDurationS = 60;
    maximalGreenLightDurationS = 120;
  }
};

struct TrafficLightsSingleFullSpeedNotRandomized
    : TSP::tsp_traffic_lights_data {
  TrafficLightsSingleFullSpeedNotRandomized() {
    useRandomizedInputs = false;
    trafficLightsCount = 1;
    spacingPercent = 0.5;
    optimalSpeedPercentOfMaxSpeed = 1.0;
    redLightDurationPercent = 1.0 / 3.0;
  }
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

struct DataNaSchP0 : DataNaSch {
  DataNaSchP0() { velocityDecreaseProbability = 0; }
};

struct DataNaSchP005 : DataNaSch {
  DataNaSchP005() { velocityDecreaseProbability = 0.05; }
};

struct DataNaSchP01 : DataNaSch {
  DataNaSchP01() { velocityDecreaseProbability = 0.1; }
};

struct DataNaSchP02 : DataNaSch {
  DataNaSchP02() { velocityDecreaseProbability = 0.2; }
};

struct DataNaSchP03 : DataNaSch {
  DataNaSchP03() { velocityDecreaseProbability = 0.3; }
};

struct DataNaSchP04 : DataNaSch {
  DataNaSchP04() { velocityDecreaseProbability = 0.4; }
};

struct DataNaSchP05 : DataNaSch {
  DataNaSchP05() { velocityDecreaseProbability = 0.5; }
};

struct DataNaSchTrafficLightsRandomL1 : DataNaSch {
  DataNaSchTrafficLightsRandomL1() {
    trafficLightsData = TrafficLightsRandomized();
  }
};

struct DataNaSchTrafficLightsRandomL2 : DataNaSch {
  DataNaSchTrafficLightsRandomL2() {
    trafficLightsData = TrafficLightsRandomized();
    trafficLightsData.trafficLightsPercent = 0.02;
  }
};

struct DataNaSchTrafficLightsRandomSameDurationL2 : DataNaSch {
  DataNaSchTrafficLightsRandomSameDurationL2() {
    trafficLightsData = TrafficLightsRandomized();
    trafficLightsData.trafficLightsPercent = 0.02;
    trafficLightsData.minimalRedLightDurationS = 60;
    trafficLightsData.maximalRedLightDurationS = 60;
    trafficLightsData.minimalGreenLightDurationS = 60;
    trafficLightsData.maximalGreenLightDurationS = 60;
  }
};

struct DataNaSchTrafficLightsOneFullSpeed : DataNaSch {
  DataNaSchTrafficLightsOneFullSpeed() {
    trafficLightsData = TrafficLightsSingleFullSpeedNotRandomized();
  }
};

struct DataNaSchTrafficLightsThreeFullSpeed : DataNaSch {
  DataNaSchTrafficLightsThreeFullSpeed() {
    trafficLightsData = TrafficLightsSingleFullSpeedNotRandomized();
    trafficLightsData.trafficLightsCount = 3;
  }
};

struct DataNaSchTrafficLightsTwentyFullSpeed : DataNaSch {
  DataNaSchTrafficLightsTwentyFullSpeed() {
    trafficLightsData = TrafficLightsSingleFullSpeedNotRandomized();
    trafficLightsData.trafficLightsCount = 20;
  }
};

struct DataNaSchTrafficLightsThree08Speed : DataNaSch {
  DataNaSchTrafficLightsThree08Speed() {
    trafficLightsData = TrafficLightsSingleFullSpeedNotRandomized();
    trafficLightsData.trafficLightsCount = 3;
    trafficLightsData.optimalSpeedPercentOfMaxSpeed = 0.8;
  }
};

struct DataNaSchTrafficLightsThree06Speed : DataNaSch {
  DataNaSchTrafficLightsThree06Speed() {
    trafficLightsData = TrafficLightsSingleFullSpeedNotRandomized();
    trafficLightsData.trafficLightsCount = 3;
    trafficLightsData.optimalSpeedPercentOfMaxSpeed = 0.6;
  }
};

struct DataNaSchTrafficLightsThree04Speed : DataNaSch {
  DataNaSchTrafficLightsThree04Speed() {
    trafficLightsData = TrafficLightsSingleFullSpeedNotRandomized();
    trafficLightsData.trafficLightsCount = 3;
    trafficLightsData.optimalSpeedPercentOfMaxSpeed = 0.4;
  }
};

struct DataNaSchTrafficLightsThree08SpeedA025
    : DataNaSchTrafficLightsThree08Speed {
  DataNaSchTrafficLightsThree08SpeedA025() { autonomousCarsPercent = 0.25; }
};

struct DataNaSchTrafficLightsThree08SpeedA05
    : DataNaSchTrafficLightsThree08Speed {
  DataNaSchTrafficLightsThree08SpeedA05() { autonomousCarsPercent = 0.5; }
};

struct DataNaSchTrafficLightsThree08SpeedA075
    : DataNaSchTrafficLightsThree08Speed {
  DataNaSchTrafficLightsThree08SpeedA075() { autonomousCarsPercent = 0.75; }
};

struct DataNaSchTrafficLightsThree08SpeedA1
    : DataNaSchTrafficLightsThree08Speed {
  DataNaSchTrafficLightsThree08SpeedA1() { autonomousCarsPercent = 1; }
};

struct DataNaSchTrafficLightsThree06SpeedA025
    : DataNaSchTrafficLightsThree08Speed {
  DataNaSchTrafficLightsThree06SpeedA025() { autonomousCarsPercent = 0.25; }
};

struct DataNaSchTrafficLightsThree06SpeedA05
    : DataNaSchTrafficLightsThree08Speed {
  DataNaSchTrafficLightsThree06SpeedA05() { autonomousCarsPercent = 0.5; }
};

struct DataNaSchTrafficLightsThree06SpeedA075
    : DataNaSchTrafficLightsThree08Speed {
  DataNaSchTrafficLightsThree06SpeedA075() { autonomousCarsPercent = 0.75; }
};

struct DataNaSchTrafficLightsThree06SpeedA1
    : DataNaSchTrafficLightsThree08Speed {
  DataNaSchTrafficLightsThree06SpeedA1() { autonomousCarsPercent = 1; }
};

struct DataNaSchTrafficLightsThree04SpeedA025
    : DataNaSchTrafficLightsThree08Speed {
  DataNaSchTrafficLightsThree04SpeedA025() { autonomousCarsPercent = 0.25; }
};

struct DataNaSchTrafficLightsThree04SpeedA05
    : DataNaSchTrafficLightsThree08Speed {
  DataNaSchTrafficLightsThree04SpeedA05() { autonomousCarsPercent = 0.5; }
};

struct DataNaSchTrafficLightsThree04SpeedA075
    : DataNaSchTrafficLightsThree08Speed {
  DataNaSchTrafficLightsThree04SpeedA075() { autonomousCarsPercent = 0.75; }
};

struct DataNaSchTrafficLightsThree04SpeedA1
    : DataNaSchTrafficLightsThree08Speed {
  DataNaSchTrafficLightsThree04SpeedA1() { autonomousCarsPercent = 1; }
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

struct DataKnospeSingleLane : DataKnospe {
  DataKnospeSingleLane() { laneCount = 1; }
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
