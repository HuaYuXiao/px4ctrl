//
// Created by hyx020222 on 7/9/24.
// last updated on 2024.08.08
//

#ifndef EASONDRONE_CONTROL_PX4CTRL_H
#define EASONDRONE_CONTROL_PX4CTRL_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <sstream>
#include <string>
#include <boost/math/special_functions.hpp>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <easondrone_msgs/ControlCommand.h>
#include "px4ctrl_utils.h"

using namespace std;

const double POS_ACCEPT = 0.2;
const double YAW_ACCEPT = 15 / 180.0 * M_PI;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
mavros_msgs::State current_state;

// odometry state
bool have_odom_;
Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;
Eigen::Quaterniond odom_orient_;
double odom_yaw_;

geometry_msgs::PoseStamped pose;
mavros_msgs::PositionTarget pos_setpoint;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandBool arm_cmd;
easondrone_msgs::ControlCommand ctrl_cmd; //无人机当前执行命令

ros::Subscriber state_sub, easondrone_ctrl_sub_, odom_sub_;
ros::Publisher local_pos_pub, setpoint_raw_local_pub, setpoint_raw_global_pub, setpoint_raw_attitude_pub_;
//变量声明 - 服务
ros::ServiceClient arming_client, set_mode_client;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void easondrone_ctrl_cb_(const easondrone_msgs::ControlCommand::ConstPtr& msg){
    ctrl_cmd = *msg;
}

// 保存无人机当前里程计信息，包括位置、速度和姿态
void odometryCallback(const nav_msgs::OdometryConstPtr &msg){
    have_odom_ = true;

    odom_pos_ << msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z;

    odom_vel_ << msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    odom_yaw_ = quaternion_to_euler(odom_orient_)[2];
}

#endif //EASONDRONE_CONTROL_PX4CTRL_H
