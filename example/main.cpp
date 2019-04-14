#include "checkerboard.h"
#include "globals.h"
#include "obstacles.h"
#include "simulation.h"
#include "simulationview.h"
#include "tslib/api.h"
#include "vehicle.h"
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <iostream>

int main(int argc, char *argv[]) {
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QGuiApplication app(argc, argv);
  QQmlApplicationEngine engine;

  TSP::tsp_simulation_result results = tspSimulate(5, 0.5, 1000, 10);
  std::cout << results.vehiclesPerTime << " " << results.vehiclesDensity
            << std::endl;
  return 0;

  Simulation simulation;

  engine.rootContext()->setContextProperty("simulation", &simulation);
  qmlRegisterType<Vehicle>("Vehicle", 1, 0, "Vehicle");
  qmlRegisterType<Checkerboard>("Checkerboard", 1, 0, "Checkerboard");
  qmlRegisterType<Obstacles>("Obstacles", 1, 0, "Obstacles");
  qmlRegisterType<SimulationView>("SimulationView", 1, 0, "SimulationView");

  engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
  if (engine.rootObjects().isEmpty())
    return -1;
  return app.exec();
}
