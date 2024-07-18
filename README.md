# px4ctrl

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2Fpx4ctrl.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-melodic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Python-3.8.10-3776AB?logo=python)
![Static Badge](https://img.shields.io/badge/Ubuntu-18.04.6-E95420?logo=ubuntu)

A ROS package to estimate and control drone.


## Installation

```shell
cd ~/EasonDrone
catkin_make install --source Modules/px4ctrl --build Modules/px4ctrl/build
```


## Launch

For p450:

```shell
roslaunch px4ctrl px4ctrl_p450_indoor.launch
```

For isis:

```shell
roslaunch px4ctrl px4ctrl_iris_indoor.launch
```

- Enter t to unlock the aircraft for takeoff.
- Enter r to return the aircraft to the takeoff point.
- Enter b to enter attitude control.
- The target point can be changed by the keypad.
- Enter l to land the aircraft autonomously.
- Enter p to print keyboard control instructions.
- Enter q to exit the program.


## Acknowledgement

Thanks to following packages:

- [prometheus_control](https://github.com/amov-lab/Prometheus/tree/v1.1/Modules/control)
- [control](https://gitee.com/robin_shaun/XTDrone/control)
