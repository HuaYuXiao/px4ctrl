# EasonDrone_Control

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2Feasondrone_control.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-melodic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Ubuntu-18.04.6-E95420?logo=ubuntu)
![Static Badge](https://img.shields.io/badge/NVIDIA-Jetson_Nano-76B900?LOGO=nvidia)

A ROS package to estimate and control drone.


## Installation

```bash
catkin_make install --source Modules/easondrone_control --build Modules/easondrone_control/build
```


## Release Note

- v3.4.2: update odometry source of gazebo
- v3.4.1: remove `message_pub`
- v3.4.0: support PID controller with `POS+VEL` loop
- v3.3.4: update `rate_hz_`
- v3.3.3: update `get_ref_pose_rviz`
- v3.3.2: support `launch`
- v3.3.1: support move mode `POS_VEL_ACC`
- v3.3.0: support move mode `XYZ_VEL`
- v3.2.1: catch invalid input of terminal control
- v2.0.0: support control type `NE`, `PID`, `UDE`
- v1.2.0: import position offset
- v1.0.0: support `VICON`


## Acknowledgement

Thanks to following packages:

- [prometheus_control](https://github.com/amov-lab/Prometheus/tree/v1.1/Modules/control)
- 
