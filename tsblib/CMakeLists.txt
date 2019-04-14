cmake_minimum_required(VERSION 3.9)
project(tslib VERSION 1.0.0 DESCRIPTION "Traffic Simulation Library")
include(GNUInstallDirs)
include(GenerateExportHeader)
add_library(tslib STATIC
  include/tslib/api.h
  include/tslib/types.h
  source/helpers/helper_id.cpp
  source/helpers/helper_id.h
  source/helpers/helper_math.cpp
  source/helpers/helper_math.h
  source/api.cpp
  source/base.h
  source/globals.h
  source/metrics.h
  source/simulation.cpp
  source/simulation.h
  ${PROJECT_BINARY_DIR}/tslib_export.h)
generate_export_header(tslib)
set_target_properties(tslib PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER include/tslib/api.h)
target_include_directories(tslib PRIVATE ${PROJECT_SOURCE_DIR})
#configure_file(tslib.pc.in tslib.pc @ONLY)
set(LIBRARY_INSTALL_DIR ${PROJECT_BINARY_DIR}/lib_install)
set(INCLUDE_INSTALL_DIR ${PROJECT_BINARY_DIR}/include_install/tslib)
target_include_directories(tslib PRIVATE .)
target_include_directories(tslib PRIVATE ./include)
target_include_directories(tslib PRIVATE ${PROJECT_BINARY_DIR})
install(TARGETS tslib DESTINATION ${LIBRARY_INSTALL_DIR})
install(FILES
 include/tslib/api.h
 include/tslib/types.h
 ${PROJECT_BINARY_DIR}/tslib_export.h DESTINATION ${INCLUDE_INSTALL_DIR}
)
