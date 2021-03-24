cmake_minimum_required(VERSION 2.8.3)
project(monocular_pose_estimator)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -pthread)
add_compile_options(-O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  geometry_msgs
  sensor_msgs
  std_msgs
  tf
  tf2
  dynamic_reconfigure
  monocular_pose_estimator_lib
)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/MonocularPoseEstimator.cfg
)

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
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs std_srvs tf tf2 monocular_pose_estimator_lib cv_bridge image_transport
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

add_library(monocular_pose_estimator      src/monocular_pose_estimator.cpp)
add_dependencies(monocular_pose_estimator ${PROJECT_NAME}_gencfg)
target_link_libraries(monocular_pose_estimator        ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pose_estimator)

add_library(nodelet      src/nodelet.cpp)
target_link_libraries(nodelet       ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} monocular_pose_estimator)

# Declare a C++ executable
add_executable(node     src/node.cpp)
target_link_libraries(node  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} nodelet monocular_pose_estimator)





