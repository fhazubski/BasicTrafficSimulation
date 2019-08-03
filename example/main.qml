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
            simulation.setTime(time)
            simulationView.refresh()
        }
    }

    Checkerboard {
        anchors.fill: parent
        step: 50
        color1: "white"
        color2: "lightgrey"
    }

    SimulationView {
        id: simulationView
        anchors.fill: parent
    }
}
