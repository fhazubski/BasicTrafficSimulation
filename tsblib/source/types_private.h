#pragma once

#include "include/tslib/types.h"
#include <vector>

namespace TSP {

struct tsp_lane_point {
  tsp_int vehicle;
  tsp_int position;
  bool hasTrafficLight;
  bool isTrafficLightRed;
  tsp_int greenLightDurationS;
  tsp_int redLightDurationS;
  tsp_int timeToNextState;
  tsp_int optimalVelocity;
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
};

struct tsp_road {
  const tsp_int lanesCount;
  const tsp_road_lane *const *lanes;
};

struct tsp_vehicle {
  tsp_id lane;
  tsp_int position;
  tsp_int velocity;
  tsp_int newVelocity;
  tsp_int greenWaveVelocity;
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

} // namespace TSP
