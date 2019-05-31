#pragma once

#include <algorithm>
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

struct tsp_road_lane {
  tsp_road_lane(const tsp_int pointsCount, const tsp_float spaceLengthM,
                const tsp_int maxVelocity, const tsp_id id)
      : pointsCount(pointsCount), spaceLengthM(spaceLengthM),
        maxVelocity(maxVelocity), id(id) {
    points.resize(pointsCount);
    std::fill(points.begin(), points.end(), 0);
  }
  const tsp_int pointsCount;
  std::vector<tsp_int> points;
  const tsp_float spaceLengthM;
  const tsp_int maxVelocity;
  const tsp_id id;
  // const tsp_position *points;
};

struct tsp_road {
  const tsp_int lanesCount;
  const tsp_road_lane *const *lanes;
};

struct tsp_rotation {
  tsp_float rotation;
};

struct tsp_vehicle_position {
  tsp_id lane;
  tsp_int position;
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

struct tsp_vehicle : tsp_vehicle_position {
  tsp_int velocity;
  tsp_int newVelocity;
  tsp_id id;
  tsp_id nextVehicle = 0;
  tsp_id previousVehicle = 0;
  tsp_float safeTimeHeadwayS = 0.0;
  bool isBreaking = false;
  bool newIsBreaking;
  tsp_int safetyGap = 0;
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
  tsp_float simulationDurationS;
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
  tsp_float simulationDurationS;
};

} // namespace TSP
