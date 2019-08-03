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

  int cellSize = width() / (m_simulation->m_roadLanePointsCount + 2);
  int xStart = (width() - cellSize * m_simulation->m_roadLanePointsCount) / 2;
  int yStart = (height() - cellSize * m_simulation->m_roadLanesCount) / 2;

  painter->setBrush(QBrush(QColor("#00000000")));
  painter->setPen("#000000");
  for (int laneIndex = 0; laneIndex < m_simulation->m_roadLanesCount;
       laneIndex++) {
    for (int pointIndex = 0; pointIndex < m_simulation->m_roadLanePointsCount;
         pointIndex++) {
      painter->drawRect(xStart + pointIndex * cellSize,
                        yStart + laneIndex * cellSize, cellSize, cellSize);
    }
  }

  painter->setBrush(QBrush(QColor("#00CCFF")));
  for (TSP::tsp_vehicle_state *vehicle = m_simulation->m_vehiclesStates;
       vehicle < m_simulation->m_vehiclesStates + m_simulation->m_vehiclesCount;
       vehicle++) {
    QRectF rect(xStart + vehicle->position * cellSize,
                yStart + vehicle->lane * cellSize, cellSize, cellSize);
    painter->drawRect(rect);
    painter->drawText(rect, Qt::AlignCenter,
                      QString::number(vehicle->velocity));
  }

  QBrush greenLight(QColor("#A3FF6A"));
  QBrush redLight(QColor("#FF5B88"));
  for (TSP::tsp_traffic_light_state *light =
           m_simulation->m_trafficLightsStates;
       light <
       m_simulation->m_trafficLightsStates + m_simulation->m_trafficLightsCount;
       light++) {
    painter->setBrush(light->isTrafficLightRed ? redLight : greenLight);
    QRectF rect(xStart + light->position * cellSize,
                yStart - cellSize * 3 / 4 + light->lane * cellSize, cellSize,
                cellSize);
    painter->setPen("#00000000");
    painter->drawRoundRect(rect, rect.x() / 2, rect.y() / 2);
    painter->setPen("#000000");
    painter->drawText(rect, Qt::AlignCenter,
                      QString::number(light->timeToNextState));
  }
}

void SimulationView::refresh() { update(); }

void SimulationView::setSimulation(Simulation *simulation) {
  m_simulation = simulation;
}
