import QtQuick 2.9
import QtQuick.Window 2.2
import Checkerboard 1.0
import Simulation 1.0
import QtQuick.Controls 1.4

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

    Row {
        spacing: 20
        Slider {
            id: speedSlider
            minimumValue: -9
            maximumValue: 9
            value: 0
            stepSize: 0.2
            width: 400
            property double speed: {
                if (value == 0) {
                    return 1;
                } else if (value < 0) {
                    return 1 + value / 10;
                }
                return 1 + value;
            }
        }
        Text {
            id: currentSpeedText
            property string part1: Math.floor(speedSlider.speed)
            property string part2: (part3.length == 1 ? "0" : "")
            property string part3: Math.floor((speedSlider.speed * 100) % 100)
            text: "Simulation speed: " + part1 + "." + part2 + part3

        }
    }


    Timer {
        interval: 1000 / speedSlider.speed
        repeat: true
        running: true
        property real time: 0
        onTriggered: {
            time += 1000
            simulation.setTime(time / 1000)
            simulationView.refresh()
        }
    }

    SimulationView {
        id: simulationView
        anchors.fill: parent
    }
}
