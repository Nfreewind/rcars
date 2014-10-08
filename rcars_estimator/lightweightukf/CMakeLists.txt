cmake_minimum_required (VERSION 2.8)
project(UKF)
add_definitions(-std=c++0x)

##################### Set some variables #####################
# Noisily default to Release build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

message(STATUS "cmake root folder:" ${CMAKE_ROOT})

##################### Find, include, and compile library #####################
find_package(Eigen REQUIRED)
find_package(Kindr REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(./include)
include_directories(${Kindr_INCLUDE_DIRS})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/gtest/")
	message(STATUS "Building GTests!")
	option(BUILD_GTEST "build gtest" ON)
	add_subdirectory(gtest gtest)
	enable_testing()
	include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
	add_executable(testState src/testState.cpp)
	target_link_libraries(testState gtest_main gtest pthread )
	add_test(testState testState)
	add_executable(testSigmaPoints src/testSigmaPoints.cpp)
	target_link_libraries(testSigmaPoints gtest_main gtest pthread )
	add_test(testSigmaPoints testSigmaPoints)
	add_executable(testUKF src/testUKF.cpp)
	target_link_libraries(testUKF gtest_main gtest pthread )
	add_test(testUKF testUKF)
endif()

# Generate FindLUKF.cmake file
file(WRITE cmake/FindLUKF.cmake
"# This file was automatically generated during the installation of the LUKF library
# and can be used through cmake to find the corresponding header files. A copy of this
# file was created in ${CMAKE_ROOT}/Modules (depending on the CMAKE_ROOT variable).

set(LUKF_INCLUDE_DIRS
${CMAKE_INSTALL_PREFIX}/include/LUKF/include
)
set(LUKF_FOUND TRUE)
message(\"-- LUKF found (include: ${CMAKE_INSTALL_PREFIX}/include/LUKF/include)\")
"
)

# Setting for make install
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/include/LUKF)")
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX}/include/LUKF)
install(FILES cmake/FindLUKF.cmake DESTINATION ${CMAKE_ROOT}/Modules)