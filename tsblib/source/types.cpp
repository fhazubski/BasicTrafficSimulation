#pragma once

#include "include/tslib/types.h"
#include "source/helpers/helper_math.h"
#include <algorithm>
#include <iostream>

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
              HelperMath::userTimeToSimulationTimeFullS(
                  trafficLightsData.minimalGreenLightDurationS) /
                  second,
              HelperMath::userTimeToSimulationTimeFullS(
                  trafficLightsData.maximalGreenLightDurationS) /
                  second) *
          second;

      point.redLightDurationS =
          HelperMath::getRandomInt(
              HelperMath::userTimeToSimulationTimeFullS(
                  trafficLightsData.minimalRedLightDurationS) /
                  second,
              HelperMath::userTimeToSimulationTimeFullS(
                  trafficLightsData.maximalRedLightDurationS) /
                  second) *
          second;

      point.isTrafficLightRed = HelperMath::getRandomInt(0, 1);
      point.timeToNextState = HelperMath::getRandomInt(
          1, (point.isTrafficLightRed ? point.redLightDurationS
                                      : point.greenLightDurationS));

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
  if (positionOffset <= 0) {
    positionOffset = 1;
  }
  tsp_float optimalVelocity = static_cast<tsp_float>(maxVelocity) *
                              trafficLightsData.optimalSpeedPercentOfMaxSpeed;
  optimalVelocity =
      std::max(static_cast<tsp_float>(1),
               std::min(static_cast<tsp_float>(maxVelocity), optimalVelocity));
  tsp_int timeOffset = static_cast<tsp_int>(std::round(
      static_cast<tsp_float>(positionOffset * second) / optimalVelocity));
#ifndef NDEBUG
  std::cout << "TSP: lights offset: " << positionOffset
            << ", time offset: " << timeOffset << ", max V: " << maxVelocity
            << ", optimal V: " << optimalVelocity << ", optimal V percent: "
            << trafficLightsData.optimalSpeedPercentOfMaxSpeed << std::endl;
#endif

  tsp_int currentPosition = HelperMath::getRandomInt(0, lane.points.size() - 1);
  tsp_float fullLapTime =
      std::round(static_cast<tsp_float>(lane.points.size()) / optimalVelocity) *
      second;
  tsp_int redLightDuration =
      static_cast<tsp_int>(
          std::round(fullLapTime * trafficLightsData.redLightDurationPercent /
                     static_cast<tsp_float>(second))) *
      second;
  tsp_int greenLightDuration =
      static_cast<tsp_int>(fullLapTime) - redLightDuration;
  tsp_int trafficLightsToCreate = trafficLightsData.trafficLightsCount;
  tsp_int currentTime = (trafficLightsToCreate + 1) * timeOffset;

#ifndef NDEBUG
  std::cout << "TSP: green light time: " << greenLightDuration
            << ", red light: " << redLightDuration << std::endl;
#endif

  lane.pointsWithTrafficLights.resize(trafficLightsToCreate);
  while (trafficLightsToCreate--) {
    tsp_int currentTimeFullSecond =
        static_cast<tsp_int>(static_cast<tsp_float>(currentTime) /
                             static_cast<tsp_float>(second)) *
        second;
    auto &point = lane.points[currentPosition % lane.points.size()];

    point.hasTrafficLight = true;
    point.greenLightDurationS = greenLightDuration;
    point.redLightDurationS = redLightDuration;
    point.optimalVelocity = static_cast<tsp_int>(std::round(optimalVelocity));
    tsp_int timeSinceGreenLightStart =
        currentTimeFullSecond % (greenLightDuration + redLightDuration);
    point.isTrafficLightRed = (timeSinceGreenLightStart >= greenLightDuration);
    if (point.isTrafficLightRed) {
      point.timeToNextState =
          redLightDuration - (timeSinceGreenLightStart - greenLightDuration);
    } else {
      point.timeToNextState = greenLightDuration - timeSinceGreenLightStart;
    }
    lane.pointsWithTrafficLights[trafficLightsToCreate] = &point;

    currentTime -= timeOffset;
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

#ifndef NDEBUG
  std::cout << "TSP: traffic lights data: " << trafficLightsData
            << ", lights randomized: "
            << (trafficLightsData != nullptr
                    ? trafficLightsData->useRandomizedInputs
                    : -1)
            << ", lights count: "
            << (trafficLightsData != nullptr
                    ? trafficLightsData->trafficLightsCount
                    : -1)
            << std::endl;
#endif

  if (trafficLightsData != nullptr && trafficLightsData->enableTrafficLights) {
    if (trafficLightsData->useRandomizedInputs) {
      initializeRandomizedTrafficLights(*this, *trafficLightsData);
    } else {
      initializeNotRandomizedTrafficLights(*this, maxVelocity,
                                           *trafficLightsData);
    }
  }
}

}; // namespace TSP
