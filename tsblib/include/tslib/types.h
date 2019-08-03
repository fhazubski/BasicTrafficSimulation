#pragma once

#include <vector>

namespace TSP {

using tsp_float = long double;
using tsp_byte = unsigned char;
using tsp_int = long long;
using tsp_id = unsigned long;

struct tsp_position {
  tsp_float x;
  tsp_float y;
};

struct tsp_line {
  tsp_position start;
  tsp_position end;
};

enum class tsp_obstacle_type {
  solid,
  road_line,
};

struct tsp_obstacle_line {
  tsp_obstacle_type type;
  tsp_line line;
};

struct tsp_traffic_lights_data {
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

struct tsp_lane_point {
  tsp_int vehicle;
  tsp_int position;
  bool hasTrafficLight;
  bool isTrafficLightRed;
  tsp_int greenLightDurationS;
  tsp_int redLightDurationS;
  tsp_int timeToNextState;
};

struct tsp_road_lane {
  tsp_road_lane(const tsp_int pointsCount, const tsp_float spaceLengthM,
                const tsp_int maxVelocity, const tsp_id id,
                const tsp_traffic_lights_data *const trafficLightsData);
  const tsp_int pointsCount;
  std::vector<tsp_lane_point> points;
  std::vector<tsp_lane_point *> pointsWithTrafficLights;
  const tsp_float spaceLengthM;
  const tsp_int maxVelocity;
  const tsp_id id;
  tsp_int vehiclesCount = 0;
  // const tsp_position *points;
};

struct tsp_road {
  const tsp_int lanesCount;
  const tsp_road_lane *const *lanes;
};

struct tsp_rotation {
  tsp_float rotation;
};

struct tsp_vehicle_state {
  tsp_id lane;
  tsp_int position;
  tsp_int velocity;
};

struct tsp_traffic_light_state {
  tsp_id lane;
  tsp_int position;
  bool isTrafficLightRed;
  tsp_int timeToNextState;
};

struct tsp_vehicle_base {
  tsp_vehicle_base() : followedRoad(nullptr) {}
  tsp_vehicle_base(const tsp_road *followedRoad) : followedRoad(followedRoad) {}
  tsp_float width;
  tsp_float height;
  tsp_float axleDistance;
  tsp_float frontWheelsDistance;
  tsp_float velocity;
  tsp_float axleAngle;
  const tsp_road *followedRoad;
  tsp_int followedLane;
};

struct tsp_engine {
  tsp_float desiredVelocity;
  tsp_float force;
};

struct tsp_steeringAxle {
  tsp_float desiredAxleAngle;
};

struct tsp_vehicle {
  tsp_id lane;
  tsp_int position;
  tsp_int velocity;
  tsp_int newVelocity;
  tsp_id id;
  tsp_id nextVehicle = 0;
  tsp_id previousVehicle = 0;
  tsp_float safeTimeHeadwayS = 0.0;
  bool isBreaking = false;
  bool newIsBreaking;
  tsp_int safetyGap = 0;
  tsp_id newLane;
  bool isAutonomous = false;
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
