#include "simulationview.h"
#include "globals.h"
#include <QBrush>
#include <QPainter>
#include <QQuickPaintedItem>
#include <iostream>

SimulationView::SimulationView(QQuickItem *parent)
    : QQuickPaintedItem(parent) {}

void SimulationView::paint(QPainter *painter) {
  if (m_simulation == nullptr)
    return;

  QBrush brush(QColor("#000000"));
  painter->setBrush(brush);

  double length = 120;
  double vehicleWidth = width() / length;
  std::cout << m_simulation->m_vehiclesPositions[0].position << std::endl;
  std::cout << static_cast<double>(
                   m_simulation->m_vehiclesPositions[0].position) *
                   vehicleWidth
            << " " << vehicleWidth << std::endl;
  for (TSP::tsp_float i = 0; i < 120; i++) {
    painter->drawLine(i * vehicleWidth, 100.0, i * vehicleWidth,
                      100.0 + vehicleWidth);
  }
  for (int i = 0; i < m_simulation->m_vehicles.size(); i++) {
    painter->fillRect(
        QRectF(
            static_cast<double>(m_simulation->m_vehiclesPositions[i].position) *
                vehicleWidth,
            100.0, vehicleWidth, vehicleWidth),
        brush);
  }
}

void SimulationView::refresh() { update(); }

void SimulationView::setSimulation(Simulation *simulation) {
  m_simulation = simulation;
}
