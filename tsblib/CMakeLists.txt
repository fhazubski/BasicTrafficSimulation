cmake_minimum_required(VERSION 3.31)
project(tslib VERSION 1.0.0 DESCRIPTION "Traffic Simulation Library")
include(GNUInstallDirs)
include(GenerateExportHeader)

set (CMAKE_CXX_STANDARD 11)

if (NOT DEFINED CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Debug)
endif()

# Qt doesn't seem to support 12.0 SDK
set(CMAKE_OSX_DEPLOYMENT_TARGET "11")

set(COMMON_SOURCES
  ${PROJECT_BINARY_DIR}/tslib_export.h
  include/tslib/api.h
  include/tslib/types.h
  source/api.cpp
  source/base.h
  source/helpers/helper_math.cpp
  source/helpers/helper_math.h
  source/metrics.h
  source/simulation.cpp
  source/simulation.h
  source/simulation_knospe.cpp
  source/simulation_knospe.h
  source/simulation_nasch.cpp
  source/simulation_nasch.h
  source/types.cpp
  source/types_private.h)
add_library(tslib STATIC ${COMMON_SOURCES})
add_executable(tests ${COMMON_SOURCES}
  tests/main.cpp
  tests/basic.cpp)
generate_export_header(tslib)
set_target_properties(tslib PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1)
if(MSVC)
  target_compile_options(tslib PRIVATE /W4)
endif()

target_include_directories(tslib PRIVATE ${PROJECT_SOURCE_DIR})
#configure_file(tslib.pc.in tslib.pc @ONLY)
set(LIBRARY_INSTALL_DIR ${PROJECT_SOURCE_DIR}/../install/tslib/bin)
set(INCLUDE_INSTALL_DIR ${PROJECT_SOURCE_DIR}/../install/tslib/include)
target_include_directories(tslib PRIVATE . ./include ${PROJECT_BINARY_DIR})
target_include_directories(tests PRIVATE . ./include ${PROJECT_BINARY_DIR})
install(TARGETS tslib DESTINATION ${LIBRARY_INSTALL_DIR})
install(FILES
  include/tslib/api.h
  include/tslib/types.h
  ${PROJECT_BINARY_DIR}/tslib_export.h DESTINATION ${INCLUDE_INSTALL_DIR})

add_custom_command(
    TARGET tslib POST_BUILD
    COMMAND ${CMAKE_COMMAND} --install ${CMAKE_BINARY_DIR} --config ${CMAKE_BUILD_TYPE}
)

add_subdirectory(third_party)
target_link_libraries(tests lib_gtest)
