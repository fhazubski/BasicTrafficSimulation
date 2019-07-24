cmake_minimum_required(VERSION 3.9)
project(tslib VERSION 1.0.0 DESCRIPTION "Traffic Simulation Library")
include(GNUInstallDirs)
include(GenerateExportHeader)
set(COMMON_SOURCES
  ${PROJECT_BINARY_DIR}/tslib_export.h
  include/tslib/api.h
  include/tslib/types.h
  source/api.cpp
  source/base.h
  source/globals.h
  source/helpers/helper_id.cpp
  source/helpers/helper_id.h
  source/helpers/helper_math.cpp
  source/helpers/helper_math.h
  source/metrics.h
  source/simulation.cpp
  source/simulation.h
  source/simulation_knospe.cpp
  source/simulation_knospe.h
  source/types.cpp)
add_library(tslib STATIC ${COMMON_SOURCES})
add_executable(tests ${COMMON_SOURCES}
  tests/main.cpp)
generate_export_header(tslib)
set_target_properties(tslib PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1
  PUBLIC_HEADER include/tslib/api.h)
if(MSVC)
  target_compile_options(tslib PRIVATE /W4)
endif()
target_include_directories(tslib PRIVATE ${PROJECT_SOURCE_DIR})
#configure_file(tslib.pc.in tslib.pc @ONLY)
set(LIBRARY_INSTALL_DIR ${PROJECT_BINARY_DIR}/lib_install)
set(INCLUDE_INSTALL_DIR ${PROJECT_BINARY_DIR}/include_install/tslib)
target_include_directories(tslib PRIVATE . ./include ${PROJECT_BINARY_DIR})
target_include_directories(tests PRIVATE . ./include ${PROJECT_BINARY_DIR})
install(TARGETS tslib DESTINATION ${LIBRARY_INSTALL_DIR})
install(FILES
  include/tslib/api.h
  include/tslib/types.h
  ${PROJECT_BINARY_DIR}/tslib_export.h DESTINATION ${INCLUDE_INSTALL_DIR})
