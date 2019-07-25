#pragma once

#include "include/tslib/types.h"
#include "source/helpers/helper_math.h"
#include <algorithm>

namespace TSP {

void initializeRandomizedTrafficLights(
    tsp_road_lane &lane, const tsp_traffic_lights_data &trafficLightsData) {
  if (trafficLightsData.trafficLightsPercent > 0) {
    tsp_int trafficLightsToCreate = static_cast<tsp_int>(
        std::round(static_cast<tsp_float>(lane.points.size()) *
                   trafficLightsData.trafficLightsPercent));
    if (trafficLightsToCreate == 0) {
      trafficLightsToCreate = 1;
    }
    lane.pointsWithTrafficLights.resize(trafficLightsToCreate);
    while (trafficLightsToCreate > 0) {
      auto &point =
          lane.points[HelperMath::getRandomInt(0, lane.points.size() - 1)];
      if (point.hasTrafficLight) {
        continue;
      }

      point.hasTrafficLight = true;

      point.greenLightDurationS =
          HelperMath::getRandomInt(
              HelperMath::userTimeToSimulationTimeS(
                  trafficLightsData.minimalGreenLightDurationS) /
                  second,
              HelperMath::userTimeToSimulationTimeS(
                  trafficLightsData.maximalGreenLightDurationS) /
                  second) *
          second;

      point.redLightDurationS =
          HelperMath::getRandomInt(
              HelperMath::userTimeToSimulationTimeS(
                  trafficLightsData.minimalRedLightDurationS) /
                  second,
              HelperMath::userTimeToSimulationTimeS(
                  trafficLightsData.maximalRedLightDurationS) /
                  second) *
          second;

      point.isTrafficLightRed = HelperMath::getRandomInt(0, 1);
      point.timeToNextState =
          (point.isTrafficLightRed ? point.redLightDurationS
                                   : point.greenLightDurationS);

      trafficLightsToCreate--;
      lane.pointsWithTrafficLights[trafficLightsToCreate] = &point;
    }
  }
}

void initializeNotRandomizedTrafficLights(
    tsp_road_lane &lane, const tsp_int maxVelocity,
    const tsp_traffic_lights_data &trafficLightsData) {

  if (trafficLightsData.trafficLightsCount <= 0) {
    return;
  }

  tsp_int positionOffset =
      lane.points.size() / trafficLightsData.trafficLightsCount;
  positionOffset = static_cast<tsp_int>(static_cast<tsp_float>(positionOffset) *
                                        trafficLightsData.spacingPercent);
  if (positionOffset == 0) {
    positionOffset = 1;
  }
  tsp_float expectedOptimalVelocity =
      static_cast<tsp_float>(maxVelocity) *
      trafficLightsData.optimalSpeedPercentOfMaxSpeed;
  tsp_float optimalVelocity = std::max(
      static_cast<tsp_float>(1),
      std::min(static_cast<tsp_float>(maxVelocity), expectedOptimalVelocity));
  tsp_int timeOffset = static_cast<tsp_int>(
      std::round(static_cast<tsp_float>(positionOffset) / optimalVelocity));

  tsp_int currentPosition = HelperMath::getRandomInt(0, lane.points.size() - 1);
  tsp_int currentTime = 0;
  tsp_int greenLightDuration = HelperMath::userTimeToSimulationTimeS(
      trafficLightsData.greenLightDurationS);
  tsp_int redLightDuration = HelperMath::userTimeToSimulationTimeS(
      trafficLightsData.redLightDurationS);
  tsp_int trafficLightsToCreate = trafficLightsData.trafficLightsCount;

  lane.pointsWithTrafficLights.resize(trafficLightsToCreate);
  while (trafficLightsToCreate--) {
    auto &point = lane.points[currentPosition % lane.points.size()];

    point.hasTrafficLight = true;
    point.greenLightDurationS = greenLightDuration;
    point.redLightDurationS = redLightDuration;
    point.timeToNextState =
        currentTime % (greenLightDuration + redLightDuration);
    point.isTrafficLightRed = (point.timeToNextState >= greenLightDuration);
    if (point.isTrafficLightRed) {
      point.timeToNextState =
          redLightDuration - (point.timeToNextState - greenLightDuration);
    } else {
      point.timeToNextState = greenLightDuration - point.timeToNextState;
    }
    lane.pointsWithTrafficLights[trafficLightsToCreate] = &point;

    currentTime += timeOffset;
    currentPosition += positionOffset;
  }
}

tsp_road_lane::tsp_road_lane(
    const tsp_int pointsCount, const tsp_float spaceLengthM,
    const tsp_int maxVelocity, const tsp_id id,
    const tsp_traffic_lights_data *const trafficLightsData)
    : pointsCount(pointsCount), spaceLengthM(spaceLengthM),
      maxVelocity(maxVelocity), id(id) {
  points.resize(pointsCount);
  for (tsp_int i = 0; i < pointsCount; i++) {
    points[i].vehicle = 0;
    points[i].position = i;
    points[i].hasTrafficLight = false;
  }

  if (trafficLightsData != nullptr) {
    if (trafficLightsData->useRandomizedInputs) {
      initializeRandomizedTrafficLights(*this, *trafficLightsData);
    } else {
      initializeNotRandomizedTrafficLights(*this, maxVelocity,
                                           *trafficLightsData);
    }
  }
}

}; // namespace TSP
