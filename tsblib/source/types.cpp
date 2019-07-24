#pragma once

#include "include/tslib/types.h"
#include "source/helpers/helper_math.h"
#include <algorithm>

namespace TSP {
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
  if ((trafficLightsData != nullptr) &&
      (trafficLightsData->trafficLightsPercent > 0)) {
    tsp_int trafficLightsToCreate = static_cast<tsp_int>(
        std::round(static_cast<tsp_float>(pointsCount) *
                   trafficLightsData->trafficLightsPercent));
    if (trafficLightsToCreate == 0) {
      trafficLightsToCreate = 1;
    }
    pointsWithTrafficLights.resize(trafficLightsToCreate);
    while (trafficLightsToCreate > 0) {
      auto &point = points[HelperMath::getRandomInt(0, pointsCount - 1)];
      if (point.hasTrafficLight) {
        continue;
      }

      point.hasTrafficLight = true;

      point.greenLightDurationS =
          HelperMath::getRandomInt(
              HelperMath::userTimeToSimulationTimeS(
                  trafficLightsData->minimalGreenLightDurationS) /
                  second,
              HelperMath::userTimeToSimulationTimeS(
                  trafficLightsData->maximalGreenLightDurationS) /
                  second) *
          second;

      point.redLightDurationS =
          HelperMath::getRandomInt(
              HelperMath::userTimeToSimulationTimeS(
                  trafficLightsData->minimalRedLightDurationS) /
                  second,
              HelperMath::userTimeToSimulationTimeS(
                  trafficLightsData->maximalRedLightDurationS) /
                  second) *
          second;

      point.isTrafficLightRed = HelperMath::getRandomInt(0, 1);
      point.timeToNextState =
          (point.isTrafficLightRed ? point.redLightDurationS
                                   : point.greenLightDurationS);

      trafficLightsToCreate--;
      pointsWithTrafficLights[trafficLightsToCreate] = &point;
    }
  }
}

}; // namespace TSP
