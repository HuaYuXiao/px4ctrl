#ifndef __CONTROLLER_UTILS_H__
#define __CONTROLLER_UTILS_H__

#include <Eigen/Eigen>
#include <math.h>
#include <numeric>

using namespace std;

struct Desired_State
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Quaterniond q;
    double yaw;
};

struct Current_State
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Quaterniond q;
    double yaw;
};

struct Ctrl_Param_PID
{
    float quad_mass;
    float tilt_angle_max;
    float hov_percent;
    Eigen::Vector3d g;
    Eigen::Vector3f int_max;
    Eigen::Matrix3d Kp;
    Eigen::Matrix3d Kv;
    Eigen::Matrix3d Kvi;
    Eigen::Matrix3d Ka;
};

struct Ctrl_Param_UDE
{
    double T_ude;
    float tilt_angle_max;
    float quad_mass;
    float hov_percent;
    Eigen::Vector3d g;
    Eigen::Vector3f int_max;
    Eigen::Matrix3d Kp;
    Eigen::Matrix3d Kd;
};

struct Ctrl_Param_NE
{
    Eigen::Matrix3d Kp;
    Eigen::Matrix3d Kd;
    double T_ude;
    double T_ne;
    float tilt_angle_max;
    float quad_mass;
    float hov_percent;
    Eigen::Vector3d g;
    Eigen::Vector3f int_max;
};

namespace controller_utils{}
#endif