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

  QBrush autonomousVehicle(QColor("#FFCC00"));
  QBrush regularVehicle(QColor("#00CCFF"));
  QBrush breakingLight(QColor("#F11250"));
  for (TSP::tsp_vehicle_state *vehicle = m_simulation->m_vehiclesStates;
       vehicle < m_simulation->m_vehiclesStates + m_simulation->m_vehiclesCount;
       vehicle++) {
    painter->setBrush(vehicle->isAutonomous ? autonomousVehicle
                                            : regularVehicle);
    QRectF rect(xStart + vehicle->position * cellSize,
                yStart + (m_simulation->m_roadLanesCount - 1 - vehicle->lane) *
                             cellSize,
                cellSize, cellSize);
    painter->drawRect(rect);
    painter->drawText(rect, Qt::AlignCenter,
                      QString::number(vehicle->velocity));
    for (int i = 1; i < vehicle->usedSpaces; i++) {
      rect.setX(rect.x() - cellSize);
      if (rect.x() < xStart) {
        rect.setX(rect.x() + cellSize * m_simulation->m_roadLanePointsCount);
      }
      rect.setWidth(cellSize);
      painter->drawRect(rect);
    }

    if (vehicle->isBreaking) {
      rect.setWidth(rect.width() / 5.0);
      painter->setBrush(breakingLight);
      painter->drawRect(rect);
    }
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
    painter->drawRoundedRect(rect, 99, 99);
    painter->setPen("#000000");
    painter->drawText(rect, Qt::AlignCenter,
                      QString::number(light->timeToNextState));
  }
}

void SimulationView::refresh() { update(); }

void SimulationView::setSimulation(Simulation *simulation) {
  m_simulation = simulation;
}
