import QtQuick 2.9
import QtQuick.Window 2.2
import Checkerboard 1.0
import Obstacles 1.0
import SimulationView 1.0

Window {
    visible: true
    width: 640
    height: 480

    onWidthChanged: {
        console.log(width, " ", height)
    }

    QtObject {
        id: globals
        readonly property real pixelsPerMeter: 9.0
        readonly property real xOffset: 0.0
        readonly property real yOffset: 1.3
    }

    Item {
        anchors.fill: parent
        Component.onCompleted: {
            simulationView.setSimulation(simulation)
            simulation.overrideAxleAngle(1, 15)
        }

        Timer {
            interval: 1000
            repeat: true
            running: true
            property real time: 0
            onTriggered: {
                time += interval
                console.log(simulation)
                simulation.setTime(time)
                simulationView.refresh()
            }
        }
    }

    Checkerboard {
        anchors.fill: parent
        step: 50
        color1: "white"
        color2: "lightgrey"
    }

    Obstacles {
        anchors.fill: parent
        meterInPixels: globals.pixelsPerMeter
        yOffset: 1.3
    }

    SimulationView {
        id: simulationView
        anchors.fill: parent
    }

    Repeater {
        model: simulation.vehicles
        Rectangle {
            id: rect
            x: (simulation.vehicles[index].x + globals.xOffset) * globals.pixelsPerMeter - width / 2
            y: (simulation.vehicles[index].y + globals.yOffset) * globals.pixelsPerMeter - height / 2
            width: simulation.vehicles[index].width * globals.pixelsPerMeter
            height: simulation.vehicles[index].height * globals.pixelsPerMeter
            rotation: 90 + simulation.vehicles[index].rotation
            color: Qt.rgba(Math.random(),Math.random(),Math.random(),1);
        }
    }
}
