# UAV control

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2Fuav_control.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-melodic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Ubuntu-18.04.6-E95420?logo=ubuntu)
![Static Badge](https://img.shields.io/badge/NVIDIA-Jetson_Nano-76B900?LOGO=nvidia)

针对阿木实验室的飞控[control](https://github.com/amov-lab/Prometheus/tree/v1.1/Modules/control)做了一些补充:

- 加入VICON定位源
- Fix some bugs for terminal control (Catch invalid input)

## How to Use?

```bash
catkin_make install --source Modules/uav_control --build build/uav_control
```
