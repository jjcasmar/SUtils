cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(SUtils VERSION 1.0.0)
set(CMAKE_COLOR_MAKEFILE ON)

# Disable in source building
if( "${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}" )
  message(FATAL "no in source building allowed." )
endif()

set(CMAKE_AUTORCC ON)

set(CMAKE_CONFIGURATION_TYPES
    Debug
    Release
    RelWithDebInfo
    MinSizeRel
    )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_AUTOMOC ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake/Modules")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake/common")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake")
include(Common)

common_find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets 3DCore 3DExtras 3DRender 3DInput)
common_find_package(OpenMesh REQUIRED SYSTEM)
common_find_package(Eigen3 REQUIRED SYSTEM)
common_find_package(CGAL REQUIRED SYSTEM)
#common_find_package(NLOPT REQUIRED SYSTEM)

common_find_package_post()

include (${CGAL_USE_FILE})

add_subdirectory(src)

include(CPackConfig)
