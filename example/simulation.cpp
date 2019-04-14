#include "simulation.h"
#include "tslib/api.h"

#include <iostream>

Simulation::Simulation(QObject *parent) : QObject(parent) {
  laneId = tspAddLane(1000);
  addVehicle(0, 0, 0, 0, 0);
  tspSetVelocityDecreaseProbability(0.5);
}

QQmlListProperty<Vehicle> Simulation::vehicles() {
  return QQmlListProperty<Vehicle>(this, this->m_vehicles);
}

TSP::tsp_obstacle_line *Simulation::obstacles() { return m_tspObstacles; }

void Simulation::setTime(qreal time) {
  tspSetTime(time);
  // std::cout << m_vehicles.count() << std::endl;
  // std::cout << time << std::endl;
  tspGetPositions(m_vehiclesPositions);
  // std::cout << m_vehiclesPositions[0].position << std::endl;
  for (int i = 0; i < m_vehicles.count(); i++) {
    m_vehicles[i]->setX(m_vehiclesPositions[i].position);
  }
}

void Simulation::addVehicle(qreal width, qreal height, qreal velocity,
                            qint32 road, qint32 lane) {
  std::cout << tspAddVehicle(laneId, 5) << std::endl;
  m_vehicles.append(new Vehicle);
  delete[] m_vehiclesPositions;
  m_vehiclesPositions = new TSP::tsp_vehicle_position[m_vehicles.count()];
  emit vehiclesChanged();
}

void Simulation::overrideAxleAngle(quint8 vehicle, qreal angle) {}
