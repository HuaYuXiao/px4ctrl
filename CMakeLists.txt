cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(PYTHON_EXECUTABLE "/usr/bin/python3.8")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
#find_package(PythonInterp REQUIRED 3)
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
        easondrone_msgs
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
        easondrone_msgs
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
)

###############################
##      声明可执行cpp文件     ##
###############################

##px4ctrl.cpp
add_executable(px4ctrl src/px4ctrl.cpp)
add_dependencies(px4ctrl ${PROJECT_NAME}_gencpp)
target_link_libraries(px4ctrl ${catkin_LIBRARIES})

##px4_pos_estimator.cpp
add_library(OptiTrackFeedbackRigidBody src/lib/OptiTrackFeedBackRigidBody.cpp)
add_executable(px4_pos_estimator src/px4_vision_pose.cpp)
add_dependencies(px4_pos_estimator ${PROJECT_NAME}_gencpp)
target_link_libraries(px4_pos_estimator
        ${catkin_LIBRARIES}
        OptiTrackFeedbackRigidBody)

add_executable(joy_node src/joy_node.cpp)
target_link_libraries(joy_node ${catkin_LIBRARIES})

add_library(KeyboardEvent src/lib/KeyboardEvent.cpp)
add_executable(terminal_control src/Utilities/terminal_control.cpp)
add_dependencies(terminal_control ${PROJECT_NAME}_gencpp)
target_link_libraries(terminal_control
        ${catkin_LIBRARIES}
        KeyboardEvent)

catkin_install_python(PROGRAMS script/multirotor_communication.py script/multirotor_keyboard_control.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
