#include "simulation.h"
#include "simulationdata.h"
#include "tslib/api.h"

#include <iostream>

Simulation::Simulation(QObject *parent) : QObject(parent) {
  //  DataNaSchTrafficLightsThree08Speed simulationData;
  //  simulationData.carDensity = 0.2;
  //  simulationData.autonomousCarsPercent = 0;
  //  simulationData.trafficLightsData.enableTrafficLights = true;
  //  simulationData.trafficLightsData.spacingPercent = 0.8;
  //  simulationData.trafficLightsData.trafficLightsCount = 5;
  //  simulationData.trafficLightsData.optimalSpeedPercentOfMaxSpeed = 0.8;
  //  simulationData.trafficLightsData.redLightDurationPercent = 0.2;
  //  simulationData.laneLengthM = 200;

  DataKnospe simulationData;
  simulationData.carDensity = 0.7;
  simulationData.vehicleOccupiedSpaceM = 3.0;
  simulationData.laneLengthM = 40;
  simulationData.maxVelocityMps = 9.0;
  simulationData.randomDecelerationMps = 9.0;
  simulationData.laneCount = 1;
  tspInitializeSimulation(simulationData);
  m_roadLanesCount = tspGetRoadLanesCount();
  m_roadLanePointsCount = tspGetRoadLanePointsCount();

  std::cout << "Points count: " << m_roadLanesCount << " X "
            << m_roadLanePointsCount << std::endl;

  m_vehiclesCount = tspGetVehiclesCount();
  m_vehiclesStates = new TSP::tsp_vehicle_state[m_vehiclesCount];
  m_trafficLightsCount = tspGetTrafficLightsCount();
  m_trafficLightsStates =
      new TSP::tsp_traffic_light_state[m_trafficLightsCount];

  setTime(0);
}

void Simulation::setTime(qreal timeS) {
  tspSetTime(timeS);
  if ((m_vehiclesStates != nullptr) && (m_trafficLightsStates != nullptr)) {
    tspGetVehicles(m_vehiclesStates);
    tspGetTrafficLights(m_trafficLightsStates);
  }
}
