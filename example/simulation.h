#ifndef SIMULATION_H
#define SIMULATION_H

#include "tslib/include/types.h"
#include <QList>
#include <QObject>
#include <QQmlListProperty>

class Simulation : public QObject {
  Q_OBJECT

public:
  explicit Simulation(QObject *parent = nullptr);

public slots:
  void setTime(qreal timeS);

public:
  TSP::tsp_int m_roadLanesCount;
  TSP::tsp_int m_roadLanePointsCount;
  TSP::tsp_int m_vehiclesCount;
  TSP::tsp_int m_trafficLightsCount;
  TSP::tsp_vehicle_state *m_vehiclesStates = nullptr;
  TSP::tsp_traffic_light_state *m_trafficLightsStates = nullptr;
};

#endif // SIMULATION_H
