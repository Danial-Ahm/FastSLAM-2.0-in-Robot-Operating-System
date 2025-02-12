cmake_minimum_required(VERSION 3.0.2)
project(fastslam)

## Compile as C++17
add_compile_options(-std=c++17)

## Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  std_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


## Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs tf visualization_msgs
  DEPENDS EIGEN3
)

## Include headers
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Add the FileReader executable

## Add the FastSLAM executable
add_executable(original src/original.cpp)
target_link_libraries(original ${catkin_LIBRARIES})

## Install targets
install(TARGETS original
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
