#pragma once
#include "tslib/types.h"
#include "tslib_export.h"

void TSLIB_EXPORT tspInitializeSimulation(TSP::tsp_simulation_data_nasch data);

void TSLIB_EXPORT tspInitializeSimulation(TSP::tsp_simulation_data_knospe data);

TSP::tsp_simulation_result TSLIB_EXPORT
tspGatherResults(TSP::tsp_float simulationDurationS);

bool TSLIB_EXPORT tspSetTime(TSP::tsp_float time);

TSP::tsp_int TSLIB_EXPORT tspGetRoadLanesCount();

TSP::tsp_int TSLIB_EXPORT tspGetRoadLanePointsCount();

TSP::tsp_int TSLIB_EXPORT tspGetVehiclesCount();

void TSLIB_EXPORT tspGetVehicles(TSP::tsp_vehicle_state *vehicleState);

TSP::tsp_int TSLIB_EXPORT tspGetTrafficLightsCount();

void TSLIB_EXPORT
tspGetTrafficLights(TSP::tsp_traffic_light_state *trafficLightState);
