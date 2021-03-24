cmake_minimum_required(VERSION 2.8.3)
project(monocular_pose_estimator_lib)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -pthread)
add_compile_options(-O3)

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

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
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
add_library(combinations      src/combinations.cpp)
target_link_libraries(combinations        ${catkin_LIBRARIES} )

add_library(led_detector           src/led_detector.cpp)
target_link_libraries(led_detector        ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_library(p3p      src/p3p.cpp)
target_link_libraries(p3p                 ${catkin_LIBRARIES} )

add_library(pose_estimator      src/pose_estimator.cpp)
target_link_libraries(pose_estimator      ${catkin_LIBRARIES} p3p led_detector combinations visualization)

add_library(visualization      src/visualization.cpp)
target_link_libraries(visualization       ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})



