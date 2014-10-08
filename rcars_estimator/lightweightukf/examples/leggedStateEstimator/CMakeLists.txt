cmake_minimum_required (VERSION 2.8)
project(LeggedStateEstimator)
add_definitions(-std=c++0x)

##################### Set some variables #####################
SET(CMAKE_BUILD_TYPE "Debug")
message(STATUS "cmake root folder:" ${CMAKE_ROOT})

##################### Find, include, and compile library #####################
find_package(Eigen REQUIRED)
find_package(Kindr REQUIRED)
message(STATUS "Eigen Include Directory: ${EIGEN_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}")
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(.)
include_directories(../../include)
include_directories(${Kindr_INCLUDE_DIRS})

add_executable(LeggedStateEstimator main.cpp LeggedStateEstimatorManager.cpp FilterVel.cpp)