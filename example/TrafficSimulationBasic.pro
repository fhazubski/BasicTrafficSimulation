TEMPLATE = app
TARGET = TrafficSimulationBasic
QT += quick qml

SOURCES += \
    main.cpp \
    simulation.cpp \
    checkerboard.cpp \
    simulationview.cpp \
    globals.cpp

RESOURCES += qml.qrc

include(tslib.pri)

HEADERS += \
    simulation.h \
    simulationdata.h \
    checkerboard.h \
    simulationview.h \
    globals.h
