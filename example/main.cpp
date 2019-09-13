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
#define SIMULATE_QUICK 0

struct simulationResultCompare {
  inline bool operator()(const TSP::tsp_simulation_result &result1,
                         const TSP::tsp_simulation_result &result2) {
    return (result1.vehiclesDensity < result2.vehiclesDensity);
  }
};

template <typename T> void simulateAndSave(const char *filename, T data) {
  std::cout << "Generating " << filename << std::endl;
  std::ofstream outCsv;
  outCsv.open(filename);
  std::vector<TSP::tsp_simulation_result> results;
  for (TSP::tsp_float d = 0.8; d >= 0.01; d -= (SIMULATE_QUICK ? 0.05 : 0.01)) {
    data.carDensity = d;
    for (int n = 0; n < (SIMULATE_QUICK ? 3 : 10); n++) {
      tspInitializeSimulation(data);
      results.push_back(tspGatherResults((SIMULATE_QUICK ? 120 : 1000)));
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
    /* NaSch default, compare p - velocity decrease probability parameter */
    //    simulateAndSave("DataNaSchP0.csv", DataNaSchP0());
    //    simulateAndSave("DataNaSchP005.csv", DataNaSchP005());
    //    simulateAndSave("DataNaSchP01.csv", DataNaSchP01());
    //    simulateAndSave("DataNaSchP02.csv", DataNaSchP02());
    //    simulateAndSave("DataNaSchP03.csv", DataNaSchP03());
    //    simulateAndSave("DataNaSchP04.csv", DataNaSchP04());
    //    simulateAndSave("DataNaSchP05.csv", DataNaSchP05());

    /* NaSch - Knospe comparison */
    simulateAndSave("DataNaSch.csv", DataNaSch());
    simulateAndSave("DataNaSchLikeKnospe.csv", DataNaSchLikeKnospe());
    simulateAndSave("DataKnospe.csv", DataKnospe());
    simulateAndSave("DataKnospeLikeNaSch.csv", DataKnospeLikeNaSch());
    simulateAndSave("DataKnospeSingleLane.csv", DataKnospeSingleLane());
    simulateAndSave("DataKnospeWithoutLaneChanging.csv",
                    DataKnospeWithoutLaneChanging());
    simulateAndSave("KnospeWithoutRandomizedBreaking.csv",
                    DataKnospeWithoutRandomizedBreaking());

    /* NaSch with randomized traffic lights */
    //    simulateAndSave("DataNaSch.csv", DataNaSch());
    //    simulateAndSave("DataNaSchTrafficLightsRandomL1.csv",
    //                    DataNaSchTrafficLightsRandomL1());
    //    simulateAndSave("DataNaSchTrafficLightsRandomL2.csv",
    //                    DataNaSchTrafficLightsRandomL2());
    //    simulateAndSave("DataNaSchTrafficLightsRandomSameDurationL2.csv",
    //                    DataNaSchTrafficLightsRandomSameDurationL2());

    /* NaSch with not randomized traffic lights - lights count comparison */
    //    simulateAndSave("DataNaSch.csv", DataNaSch());
    //    simulateAndSave("DataNaSchTrafficLightsOneFullSpeed.csv",
    //                    DataNaSchTrafficLightsOneFullSpeed());
    //    simulateAndSave("DataNaSchTrafficLightsThreeFullSpeed.csv",
    //                    DataNaSchTrafficLightsThreeFullSpeed());
    //    simulateAndSave("DataNaSchTrafficLightsTwentyFullSpeed.csv",
    //                    DataNaSchTrafficLightsTwentyFullSpeed());

    /* NaSch with not randomized traffic lights - optimal speed comparison */
    //    simulateAndSave("DataNaSch.csv", DataNaSch());
    //    simulateAndSave("DataNaSchTrafficLightsThreeFullSpeed.csv",
    //                    DataNaSchTrafficLightsThreeFullSpeed());
    //    simulateAndSave("DataNaSchTrafficLightsThree08Speed.csv",
    //                    DataNaSchTrafficLightsThree08Speed());
    //    simulateAndSave("DataNaSchTrafficLightsThree06Speed.csv",
    //                    DataNaSchTrafficLightsThree06Speed());
    //    simulateAndSave("DataNaSchTrafficLightsThree04Speed.csv",
    //                    DataNaSchTrafficLightsThree04Speed());

    /* NaSch with traffic lights and autonomous vehicles comparison */
    //    simulateAndSave("DataNaSchTrafficLightsThree08Speed.csv",
    //                    DataNaSchTrafficLightsThree08Speed());
    //    simulateAndSave("DataNaSchTrafficLightsThree08SpeedA025.csv",
    //                    DataNaSchTrafficLightsThree08SpeedA025());
    //    simulateAndSave("DataNaSchTrafficLightsThree08SpeedA05.csv",
    //                    DataNaSchTrafficLightsThree08SpeedA05());
    //    simulateAndSave("DataNaSchTrafficLightsThree08SpeedA075.csv",
    //                    DataNaSchTrafficLightsThree08SpeedA075());
    //    simulateAndSave("DataNaSchTrafficLightsThree08SpeedA1.csv",
    //                    DataNaSchTrafficLightsThree08SpeedA1());
    //    simulateAndSave("DataNaSchTrafficLightsThree06Speed.csv",
    //                    DataNaSchTrafficLightsThree06Speed());
    //    simulateAndSave("DataNaSchTrafficLightsThree06SpeedA025.csv",
    //                    DataNaSchTrafficLightsThree06SpeedA025());
    //    simulateAndSave("DataNaSchTrafficLightsThree06SpeedA05.csv",
    //                    DataNaSchTrafficLightsThree06SpeedA05());
    //    simulateAndSave("DataNaSchTrafficLightsThree06SpeedA075.csv",
    //                    DataNaSchTrafficLightsThree06SpeedA075());
    //    simulateAndSave("DataNaSchTrafficLightsThree06SpeedA1.csv",
    //                    DataNaSchTrafficLightsThree06SpeedA1());
    //    simulateAndSave("DataNaSchTrafficLightsThree04Speed.csv",
    //                    DataNaSchTrafficLightsThree04Speed());
    //    simulateAndSave("DataNaSchTrafficLightsThree04SpeedA025.csv",
    //                    DataNaSchTrafficLightsThree04SpeedA025());
    //    simulateAndSave("DataNaSchTrafficLightsThree04SpeedA05.csv",
    //                    DataNaSchTrafficLightsThree04SpeedA05());
    //    simulateAndSave("DataNaSchTrafficLightsThree04SpeedA075.csv",
    //                    DataNaSchTrafficLightsThree04SpeedA075());
    //    simulateAndSave("DataNaSchTrafficLightsThree04SpeedA1.csv",
    //                    DataNaSchTrafficLightsThree04SpeedA1());

    std::cout << "Results generated successfully" << std::endl;
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
