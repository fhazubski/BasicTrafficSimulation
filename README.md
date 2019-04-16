# BasicTrafficSimulation
(Work in progress) The simpler version of the application to simulate traffic flow scenarios. The goal is to compare efficiency of AI controlled vehicles to those controlled manually.

This project is developed in a form of a standalone static C++ library (tslib).

## Example setup
Below you can find the setup I have been using.

#### Tools
* Microsoft Visual Studio Community 2017
* CMake 3
* Qt Creator 4.8.2
* Qt 5.9.7 MSVC2017 64bit

#### Steps to run the example application
* Generate project files with cmake. Simply run `tsblib/generate_solution_VS2017x64.cmd`
* Build the project, build INSTALL target. This will copy library binaries and headers to the path expected by the example Qt project.
* Open example Qt project. Open `example/TrafficSimulationBasic.pro` with Qt Creator.
* Build and run the project.

#### Notes
* If you intend to use the library with Qt, have a look at `example/tslib.pri` file. It contains the example configuration needed to use the library in the Qt project.
