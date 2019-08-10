import QtQuick 2.9
import QtQuick.Window 2.2
import Checkerboard 1.0
import Simulation 1.0

Window {
    visible: true
    width: 640
    height: 480

    Simulation {
        id: simulation
        Component.onCompleted: {
            simulationView.setSimulation(simulation)
        }
    }

    Timer {
        interval: 1000
        repeat: true
        running: true
        property real time: 0
        onTriggered: {
            time += interval
            simulation.setTime(time / 1000)
            simulationView.refresh()
        }
    }

    SimulationView {
        id: simulationView
        anchors.fill: parent
    }
}
