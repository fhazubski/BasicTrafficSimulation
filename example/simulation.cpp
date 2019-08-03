#include "simulation.h"
#include "simulationdata.h"
#include "tslib/api.h"

#include <iostream>

Simulation::Simulation(QObject *parent) : QObject(parent) {
  DataNaSchTrafficLightsThree08Speed simulationData;
  simulationData.carDensity = 0.1;
  simulationData.laneLengthM = 200;
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
}

void Simulation::setTime(qreal time) {
  tspSetTime(time);
  if ((m_vehiclesStates != nullptr) && (m_trafficLightsStates != nullptr)) {
    tspGetVehicles(m_vehiclesStates);
    tspGetTrafficLights(m_trafficLightsStates);
  }
}
