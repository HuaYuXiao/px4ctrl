cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(PYTHON_EXECUTABLE "/usr/bin/python3")
#find_package(PythonInterp REQUIRED 3)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        mavros
        tf
        tf2_ros
        tf2_eigen
        message_filters
        tf2_geometry_msgs
        quadrotor_msgs
        mavros_msgs
        geometry_msgs
        std_msgs
        nav_msgs
        sensor_msgs
        easondrone_msgs
)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        include
)

add_executable(px4ctrl_control src/PX4CtrlFSM.cpp)
target_link_libraries(px4ctrl_control ${catkin_LIBRARIES})

add_executable(px4ctrl_vision src/px4ctrl_vision.cpp)
target_link_libraries(px4ctrl_vision ${catkin_LIBRARIES})

add_executable(px4ctrl_terminal src/px4ctrl_terminal.cpp)
target_link_libraries(px4ctrl_terminal ${catkin_LIBRARIES})

add_executable(joy_node src/joy_node.cpp)
target_link_libraries(joy_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/px4ctrl_navigate.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
