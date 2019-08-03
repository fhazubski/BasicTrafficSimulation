#include "checkerboard.h"
#include "globals.h"
#include "simulation.h"
#include "simulationdata.h"
#include "simulationview.h"
#include "tslib/api.h"
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <algorithm>
#include <fstream>
#include <iostream>

#define SIMULATE_AND_SAVE_TO_FILE 0
#define SIMULATE_QUICK 1
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
  for (TSP::tsp_float d = 0.8; d >= 0.01; d -= (SIMULATE_QUICK ? 0.05 : 0.01)) {
    // std::cout << "progress: " << d << std::endl;
    data.carDensity = d;
    for (int n = 0; n < (SIMULATE_QUICK ? 10 : 10); n++) {
      tspInitializeSimulation(data);
      results.push_back(tspGatherResults((SIMULATE_QUICK ? 200 : 1000)));
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
    /*simulateAndSave("NaSchP01.csv", DataNaSch());
    simulateAndSave("NaSchP05.csv", DataNaSchP05());
    simulateAndSave("NaSchLikeKnospe.csv", DataNaSchLikeKnospe());
    simulateAndSave("KnospeWithoutLaneChanging.csv",
                    DataKnospeWithoutLaneChanging());*/
    /*simulateAndSave("NaSchP0.csv", DataNaSchP0());
    simulateAndSave("Knospe.csv", DataKnospe());
    simulateAndSave("KnospeWithoutRandomizedBreaking.csv",
                    DataKnospeWithoutRandomizedBreaking());*/
    /*
    simulateAndSave("NaSchP03Auto0.csv", DataNaSchP03Auto0());
    simulateAndSave("NaSchP03Auto025.csv", DataNaSchP03Auto025());
    simulateAndSave("NaSchP03Auto05.csv", DataNaSchP03Auto05());
    simulateAndSave("NaSchP03Auto075.csv", DataNaSchP03Auto075());
    simulateAndSave("NaSchP03Auto1.csv", DataNaSchP03Auto1());
    simulateAndSave("NaSchP03.csv", DataNaSchP03Auto0());
    simulateAndSave("NaSchP0225.csv", DataNaSchP0225());
    simulateAndSave("NaSchP015.csv", DataNaSchP015());
    simulateAndSave("NaSchP0075.csv", DataNaSchP0075());
    simulateAndSave("NaSchP0.csv", DataNaSchP0());*/
    simulateAndSave("DataNaSch.csv", DataNaSch());
    simulateAndSave("DataNaSchTrafficLightsP001.csv",
                    DataNaSchTrafficLightsP001());
    simulateAndSave("DataNaSchTrafficLightsP002.csv",
                    DataNaSchTrafficLightsP002());

    simulateAndSave("DataNaSchTrafficLightsOneFullSpeed.csv",
                    DataNaSchTrafficLightsOneFullSpeed());
    simulateAndSave("DataNaSchTrafficLightsThreeFullSpeed.csv",
                    DataNaSchTrafficLightsThreeFullSpeed());
    simulateAndSave("DataNaSchTrafficLightsTwentyFullSpeed.csv",
                    DataNaSchTrafficLightsTwentyFullSpeed());
    simulateAndSave("DataNaSchTrafficLightsOne08Speed.csv",
                    DataNaSchTrafficLightsOne08Speed());
    simulateAndSave("DataNaSchTrafficLightsThree08Speed.csv",
                    DataNaSchTrafficLightsThree08Speed());
    simulateAndSave("DataNaSchTrafficLightsTwenty08Speed.csv",
                    DataNaSchTrafficLightsTwenty08Speed());

    return 0;
  }

  qmlRegisterType<Checkerboard>("Checkerboard", 1, 0, "Checkerboard");
  qmlRegisterType<Simulation>("Simulation", 1, 0, "Simulation");
  qmlRegisterType<SimulationView>("Simulation", 1, 0, "SimulationView");

  engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
  if (engine.rootObjects().isEmpty())
    return -1;
  return app.exec();
}
