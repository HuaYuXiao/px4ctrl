cmake_minimum_required(VERSION 2.8.3)
project(uav_control)

# Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++14)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros
  std_msgs
  std_srvs
  tf
  tf2_ros
  tf2_eigen
        tf2_geometry_msgs
  mavros_msgs
  prometheus_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs 
  nav_msgs
  sensor_msgs
  std_msgs
        prometheus_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${catkin_INCLUDE_DIRS}
        include
        include/Position_Controller
        $ENV{HOME}/Prometheus/Modules/common/include
        $ENV{HOME}/Prometheus/Modules/ground_station/include
)


###############################
##      声明可执行cpp文件     ##
###############################

##px4_pos_controller.cpp
add_executable(px4_pos_controller src/px4_pos_controller.cpp)
add_dependencies(px4_pos_controller prometheus_control_gencpp) 
target_link_libraries(px4_pos_controller ${catkin_LIBRARIES})

##px4_pos_estimator.cpp
add_library(OptiTrackFeedbackRigidBody src/lib/OptiTrackFeedBackRigidBody.cpp)
add_executable(px4_pos_estimator src/px4_pos_estimator.cpp)
add_dependencies(px4_pos_estimator prometheus_control_gencpp)
target_link_libraries(px4_pos_estimator ${catkin_LIBRARIES})
target_link_libraries(px4_pos_estimator OptiTrackFeedbackRigidBody)

add_executable(joy_node src/Utilities/joy_node.cpp)
target_link_libraries(joy_node ${catkin_LIBRARIES})

add_library(KeyboardEvent src/lib/KeyboardEvent.cpp)
add_executable(terminal_control src/Utilities/terminal_control.cpp)
add_dependencies(terminal_control prometheus_control_gencpp)
target_link_libraries(terminal_control ${catkin_LIBRARIES})
target_link_libraries(terminal_control KeyboardEvent)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
