# px4ctrl

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2Fpx4ctrl.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-melodic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Python-3.8.10-3776AB?logo=python)
![Static Badge](https://img.shields.io/badge/Ubuntu-18.04.6-E95420?logo=ubuntu)

A ROS package as an interface for drone control with PX4 via MAVROS. Implement following functions: 

1. control drone to Arm, Takeoff, Offboard, Move ,Hold, Land, Return, etc. Publish to `/mavros/setpoint_raw/local`
2. process location data from SLAM, Gazebo, MoCap, etc. Publish to `/mavros/vision_pose/pose`
3. process control command from path-planner, with format `quadrotor_msgs::PositionCommand`

![Snipaste_2024-08-08_11-42-16.png](doc%2FSnipaste_2024-08-08_11-42-16.png)

## Installation

```shell
cd ~/EasonDrone
catkin_make install --source Modules/px4ctrl --build Modules/px4ctrl/build
```

## Launch

```shell
roslaunch px4ctrl px4ctrl.launch
```

## Acknowledgement

Thanks for following packages:

- [Prometheus/prometheus_control](https://github.com/amov-lab/Prometheus/Modules/control)
- [XTDrone/control](https://gitee.com/robin_shaun/XTDrone/control)
- [Fast-Drone-250/px4ctrl](https://github.com/ZJU-FAST-Lab/Fast-Drone-250/src/realflight_modules/px4ctrl)

Related docs:

- [mavros wiki](https://wiki.ros.org/mavros)
- [px4 user guide](https://docs.px4.io/master/en/)
