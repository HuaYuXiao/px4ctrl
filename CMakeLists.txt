cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(PYTHON_EXECUTABLE "/usr/bin/python3.8")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
#find_package(PythonInterp REQUIRED 3)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros
        message_filters
  std_srvs
  tf
  tf2_ros
  tf2_eigen
        tf2_geometry_msgs
  mavros_msgs
        geometry_msgs
        std_msgs
        nav_msgs
        sensor_msgs
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

##px4ctrl_node.cpp
add_executable(px4ctrl_node src/px4ctrl_node.cpp)
add_dependencies(px4ctrl_node ${PROJECT_NAME}_gencpp)
target_link_libraries(px4ctrl_node ${catkin_LIBRARIES})

##px4ctrl_vision.cpp
add_executable(px4ctrl_vision src/px4ctrl_vision.cpp)
add_dependencies(px4ctrl_vision ${PROJECT_NAME}_gencpp)
target_link_libraries(px4ctrl_vision ${catkin_LIBRARIES})

add_executable(joy_node src/joy_node.cpp)
target_link_libraries(joy_node ${catkin_LIBRARIES})

add_executable(px4ctrl_terminal src/px4ctrl_terminal.cpp)
add_dependencies(px4ctrl_terminal ${PROJECT_NAME}_gencpp)
target_link_libraries(px4ctrl_terminal ${catkin_LIBRARIES})

add_executable(direct_goal src/direct_goal.cpp)
add_dependencies(px4ctrl_node ${PROJECT_NAME}_gencpp)
target_link_libraries(direct_goal ${catkin_LIBRARIES})

add_executable(offb_node src/offb_node.cpp)
add_dependencies(offb_node ${PROJECT_NAME}_gencpp)
target_link_libraries(offb_node ${catkin_LIBRARIES})

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