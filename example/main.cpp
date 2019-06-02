#include "checkerboard.h"
#include "globals.h"
#include "obstacles.h"
#include "simulation.h"
#include "simulationdata.h"
#include "simulationview.h"
#include "tslib/api.h"
#include "vehicle.h"
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <algorithm>
#include <fstream>
#include <iostream>

#define SIMULATE_AND_SAVE_TO_FILE 1
#define SIMULATE_QUICK 0
#define SIMULATE_KNOSPE 0

struct simulationResultCompare {
  inline bool operator()(const TSP::tsp_simulation_result &result1,
                         const TSP::tsp_simulation_result &result2) {
    return (result1.vehiclesDensity < result2.vehiclesDensity);
  }
};

template <typename T> void simulateAndSave(const char *filename, T data) {
  std::ofstream outCsv;
  outCsv.open(filename);
  std::vector<TSP::tsp_simulation_result> results;
  data.simulationDurationS = (SIMULATE_QUICK ? 200 : 1000);
  for (TSP::tsp_float d = 0.8; d >= 0.01; d -= (SIMULATE_QUICK ? 0.05 : 0.01)) {
    std::cout << "progress: " << d << std::endl;
    data.carDensity = d;
    for (int n = 0; n < (SIMULATE_QUICK ? 10 : 10); n++) {
      results.push_back(tspSimulate(data));
    }
  }

  std::sort(results.begin(), results.end(), simulationResultCompare());

  for (auto &result : results) {
    std::string toSave = std::to_string(result.vehiclesDensity) + ";" +
                         std::to_string(result.vehiclesPerTime);
    std::replace(toSave.begin(), toSave.end(), '.', ',');
    outCsv << toSave << std::endl;
  }
  outCsv.close();
}

int main(int argc, char *argv[]) {
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QGuiApplication app(argc, argv);
  QQmlApplicationEngine engine;

  if (SIMULATE_AND_SAVE_TO_FILE) {
    simulateAndSave("NaSchP01.csv", DataNaSch());
    simulateAndSave("NaSchP05.csv", DataNaSchP05());
    simulateAndSave("NaSchLikeKnospe.csv", DataNaSchLikeKnospe());
    simulateAndSave("KnospeWithoutLaneChanging.csv",
                    DataKnospeWithoutLaneChanging());
    simulateAndSave("Knospe.csv", DataKnospe());
    return 0;
  }
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
