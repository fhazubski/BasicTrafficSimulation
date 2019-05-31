#pragma once
#include "tslib/types.h"
#include "tslib_export.h"

TSP::tsp_simulation_result TSLIB_EXPORT
tspSimulate(TSP::tsp_simulation_data_nasch data);

TSP::tsp_simulation_result TSLIB_EXPORT
tspSimulate(TSP::tsp_simulation_data_knospe data);

bool TSLIB_EXPORT tspAddVehicle(TSP::tsp_id lane, TSP::tsp_int velocity);

TSP::tsp_id TSLIB_EXPORT tspAddLane(TSP::tsp_float length);

bool TSLIB_EXPORT tspSetTime(TSP::tsp_float time);

bool TSLIB_EXPORT tspGetPositions(TSP::tsp_vehicle_position *vehiclePositions);

bool TSLIB_EXPORT tspSetVelocityDecreaseProbability(TSP::tsp_float p);
