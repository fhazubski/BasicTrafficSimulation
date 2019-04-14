#ifndef SIMULATIONVIEW_H
#define SIMULATIONVIEW_H

#include "simulation.h"
#include <QObject>
#include <QQuickPaintedItem>

class SimulationView : public QQuickPaintedItem {
  Q_OBJECT
public:
  SimulationView(QQuickItem *parent = 0);
  void paint(QPainter *painter);

public slots:
  void refresh();
  void setSimulation(Simulation *simulation);

private:
  Simulation *m_simulation;
};

#endif // SIMULATIONVIEW_H
