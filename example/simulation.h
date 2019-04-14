#ifndef SIMULATION_H
#define SIMULATION_H

#include "tslib/types.h"
#include "vehicle.h"
#include <QList>
#include <QObject>
#include <QQmlListProperty>

class Simulation : public QObject {
  Q_OBJECT
  Q_PROPERTY(
      QQmlListProperty<Vehicle> vehicles READ vehicles NOTIFY vehiclesChanged)

public:
  explicit Simulation(QObject *parent = nullptr);
  QQmlListProperty<Vehicle> vehicles();
  TSP::tsp_obstacle_line *obstacles();

public slots:
  void setTime(qreal time);
  void addVehicle(qreal width, qreal height, qreal velocity, qint32 road,
                  qint32 lane);
  void overrideAxleAngle(quint8 vehicle, qreal angle);

signals:
  void vehiclesChanged();

public:
  QList<Vehicle *> m_vehicles;
  TSP::tsp_vehicle_position *m_vehiclesPositions = nullptr;

private:
  TSP::tsp_vehicle *m_tspVehicles = nullptr;
  TSP::tsp_obstacle_line *m_tspObstacles = nullptr;
  TSP::tsp_id laneId;
};

#endif // SIMULATION_H
