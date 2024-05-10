# prometheus_control

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2Fuav_control.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-melodic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Ubuntu-18.04.6-E95420?logo=ubuntu)
![Static Badge](https://img.shields.io/badge/NVIDIA-Jetson_Nano-76B900?LOGO=nvidia)

A ROS package to control drone, modified from [prometheus_control](https://github.com/amov-lab/Prometheus/tree/v1.1/Modules/control)


## Release Note


- v3.3.1: 
  - support move mode `XYZ_VEL_ACC`
  - merge with `prometheus_station_utils` from `prometheus_station`
- v3.3.0: support move mode `XYZ_VEL` 
- v3.2.1: catch invalid input of terminal control
- v2.0.0: support control type `NE`, `PID`, `UDE`
- v1.2.0: import position offset
- 加入VICON定位源

```bash
catkin_make install --source Modules/uav_control --build build/uav_control
```
