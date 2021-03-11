cmake_minimum_required(VERSION 2.8.3)
project(happymoon_quad_control)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -pthread)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2
)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs std_srvs tf tf2
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)
add_library(geometry_eigen_conversions      src/geometry_eigen_conversions.cpp)
target_link_libraries(geometry_eigen_conversions        ${catkin_LIBRARIES})

add_library(math_common           src/math_common.cpp)
target_link_libraries(math_common        ${catkin_LIBRARIES})

add_library(control_data_arbiter      src/control_data_arbiter.cpp)
target_link_libraries(control_data_arbiter        ${catkin_LIBRARIES})

add_library(happymoon_control      src/happymoon_control.cpp)
target_link_libraries(happymoon_control       ${catkin_LIBRARIES} geometry_eigen_conversions math_common control_data_arbiter)

# Declare a C++ executable
add_executable(happymoon_quad_control_node     src/happymoon_quad_control_node.cpp)
target_link_libraries(happymoon_quad_control_node  ${catkin_LIBRARIES} happymoon_control)

