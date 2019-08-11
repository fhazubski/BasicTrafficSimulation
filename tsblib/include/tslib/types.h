#pragma once

namespace TSP {

using tsp_float = long double;
using tsp_byte = unsigned char;
using tsp_int = long long;
using tsp_id = unsigned long;

struct tsp_traffic_lights_data {
  bool enableTrafficLights = false;
  bool useRandomizedInputs;
  // Luck based inputs, used if useRandomizedInputs = true
  tsp_float trafficLightsPercent;
  tsp_float minimalGreenLightDurationS;
  tsp_float maximalGreenLightDurationS;
  tsp_float minimalRedLightDurationS;
  tsp_float maximalRedLightDurationS;

  // Not luck based inputs, used if useRandomizedInputs = false
  tsp_int trafficLightsCount;
  tsp_float spacingPercent;
  tsp_float optimalSpeedPercentOfMaxSpeed;
  tsp_float redLightDurationPercent;
};

struct tsp_vehicle_state {
  tsp_id lane;
  tsp_int position;
  tsp_int velocity;
  tsp_int usedSpaces;
  bool isBreaking;
  bool isAutonomous;
};

struct tsp_traffic_light_state {
  tsp_id lane;
  tsp_int position;
  bool isTrafficLightRed;
  tsp_int timeToNextState;
};

struct tsp_simulation_result {
  tsp_float vehiclesPerTime;
  tsp_float vehiclesDensity;
};

struct tsp_simulation_data_nasch {
  tsp_float maxVelocityMps;
  tsp_float newVehicleVelocityMps;
  tsp_float accelerationMps;
  tsp_float randomDecelerationMps;
  tsp_float velocityDecreaseProbability;
  tsp_float vehicleOccupiedSpaceM;
  tsp_float spaceLengthM;
  tsp_float laneLengthM;
  tsp_float carDensity;
  tsp_float autonomousCarsPercent;
  tsp_traffic_lights_data trafficLightsData;
};

struct tsp_simulation_data_knospe {
  tsp_float maxVelocityMps;
  tsp_float newVehicleVelocityMps;
  tsp_float accelerationMps;
  tsp_float randomDecelerationMps;
  tsp_float velocityDecreaseProbabilityB;
  tsp_float velocityDecreaseProbability0;
  tsp_float velocityDecreaseProbabilityD;
  tsp_float safeTimeHeadwayS;
  tsp_float vehicleOccupiedSpaceM;
  tsp_float spaceLengthM;
  tsp_float safetyGapM;
  tsp_int laneCount;
  tsp_float laneLengthM;
  tsp_float carDensity;
  bool allowLaneChanging = true;
};

} // namespace TSP
