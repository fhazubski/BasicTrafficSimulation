#pragma once
#include "tslib/types.h"
#include "tslib_export.h"

TSP::tsp_simulation_result TSLIB_EXPORT tspSimulateNaSch(
    TSP::tsp_float maxVelocityMps, TSP::tsp_float newVehicleVelocityMps,
    TSP::tsp_float accelerationMps, TSP::tsp_float randomDecelerationMps,
    TSP::tsp_float velocityDecreaseProbability,
    TSP::tsp_float vehicleOccupiedSpaceM, TSP::tsp_float spaceLengthM,
    TSP::tsp_float laneLengthM, TSP::tsp_float carDensity,
    TSP::tsp_float simulationDurationS);

TSP::tsp_simulation_result TSLIB_EXPORT tspSimulateKnospe(
    TSP::tsp_float maxVelocityMps, TSP::tsp_float newVehicleVelocityMps,
    TSP::tsp_float accelerationMps, TSP::tsp_float randomDecelerationMps,
    TSP::tsp_float velocityDecreaseProbabilityB,
    TSP::tsp_float velocityDecreaseProbability0,
    TSP::tsp_float velocityDecreaseProbabilityD,
    TSP::tsp_float safeTimeHeadwayS, TSP::tsp_float vehicleOccupiedSpaceM,
    TSP::tsp_float spaceLengthM, TSP::tsp_float safetyGapM,
    TSP::tsp_int laneCount, TSP::tsp_float laneLengthM,
    TSP::tsp_float carDensity, TSP::tsp_float simulationDurationS);

bool TSLIB_EXPORT tspAddVehicle(TSP::tsp_id lane, TSP::tsp_int velocity);

TSP::tsp_id TSLIB_EXPORT tspAddLane(TSP::tsp_float length);

bool TSLIB_EXPORT tspSetTime(TSP::tsp_float time);

bool TSLIB_EXPORT tspGetPositions(TSP::tsp_vehicle_position *vehiclePositions);

bool TSLIB_EXPORT tspSetVelocityDecreaseProbability(TSP::tsp_float p);
