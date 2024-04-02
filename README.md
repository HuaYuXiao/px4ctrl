# UAV control

针对阿木家的飞控[control](https://github.com/amov-lab/Prometheus/tree/v1.1/Modules/control)做了一些补充，加入VICON定位源。

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2Fuav_control.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-melodic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-11-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Ubuntu-18.04.6-E95420?logo=ubuntu)
![Static Badge](https://img.shields.io/badge/NVIDIA-Jetson_Nano-76B900?LOGO=nvidia)

## How to Use?

### 下载源码

```bash
cd ~/Prometheus/Modules
```

```bash
git clone https://github.com/HuaYuXiao/uav_control.git control
```

### 编译安装

```bash
cd ~/Prometheus
```

```bash
catkin_make install -j1 -l1 --source Modules/control --build build/control
```





