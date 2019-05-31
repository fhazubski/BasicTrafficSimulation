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
#include <algorithm>
#include <fstream>
#include <iostream>

#define SIMULATE_AND_SAVE_TO_FILE 1
#define SIMULATE_QUICK 1
#define SIMULATE_KNOSPE 1

struct simulationResultCompare {
  inline bool operator()(const TSP::tsp_simulation_result &result1,
                         const TSP::tsp_simulation_result &result2) {
    return (result1.vehiclesDensity < result2.vehiclesDensity);
  }
};

int main(int argc, char *argv[]) {
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QGuiApplication app(argc, argv);
  QQmlApplicationEngine engine;

  if (SIMULATE_AND_SAVE_TO_FILE) {
    std::ofstream outCsv;
    outCsv.open("symulacja.csv");
    std::vector<TSP::tsp_simulation_result> results;
    for (TSP::tsp_float d = 0.8; d >= 0.01;
         d -= (SIMULATE_QUICK ? 0.05 : 0.01)) {
      std::cout << "progress: " << d << std::endl;
      /*
            TSP::tsp_simulation_result TSLIB_EXPORT tspSimulate(
                TSP::tsp_float maxVelocityMps, TSP::tsp_float
         newVehicleVelocityMps, TSP::tsp_float accelerationMps, TSP::tsp_float
         randomDecelerationMps, TSP::tsp_float velocityDecreaseProbability,
                TSP::tsp_float vehicleOccupiedSpaceM, TSP::tsp_float
         spaceLengthM, TSP::tsp_float laneLengthM, TSP::tsp_float carDensity,
                TSP::tsp_float simulationDurationS);

      TSP::tsp_simulation_result TSLIB_EXPORT tspSimulateKnospe(
          TSP::tsp_float maxVelocityMps, TSP::tsp_float newVehicleVelocityMps,
          TSP::tsp_float accelerationMps, TSP::tsp_float randomDecelerationMps,
          TSP::tsp_float velocityDecreaseProbabilityB,
          TSP::tsp_float velocityDecreaseProbability0,
          TSP::tsp_float velocityDecreaseProbabilityD,
          TSP::tsp_float safeTimeHeadwayS, TSP::tsp_float vehicleOccupiedSpaceM,
          TSP::tsp_float spaceLengthM, TSP::tsp_float safetyGapM,
          TSP::tsp_int laneCount, TSP::tsp_float laneLengthM,
          TSP::tsp_float carDensity, TSP::tsp_float simulationDurationS);
      */
      for (int n = 0; n < (SIMULATE_QUICK ? 10 : 5); n++) {
        if (SIMULATE_KNOSPE) {
          results.push_back(tspSimulateKnospe(30, 30, 1, 5, 0.94, 0.5, 0.1, 6,
                                              1.5, 1.5, 10.5, 2, 1000, d * 0.33,
                                              (SIMULATE_QUICK ? 200 : 1000)));
        } else {
          results.push_back(tspSimulateNaSch(37.5, 37.5, 2.5, 2.5, 0.1, 5, 0.25,
                                             1000, d,
                                             (SIMULATE_QUICK ? 2000 : 1000)));
        }
      }
    }

    std::sort(results.begin(), results.end(), simulationResultCompare());

    for (auto &result : results) {
      std::string toSave = std::to_string(result.vehiclesDensity * 3.0) + ";" +
                           std::to_string(result.vehiclesPerTime);
      std::replace(toSave.begin(), toSave.end(), '.', ',');
      outCsv << toSave << std::endl;
    }
    outCsv.close();
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
