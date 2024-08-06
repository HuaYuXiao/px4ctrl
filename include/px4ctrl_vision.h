#ifndef PX4CTRL_PX4_VISION_H
#define PX4CTRL_PX4_VISION_H

/***************************************************************************************************************************
 * px4_vision_pose.h
 *
 * Author: Qyp
* Maintainer: Eason Hua
* Update Time: 2024.08.05
 *
 * 说明: mavros位置估计程序
 *      1. 订阅Mocap设备 (vrpn-client-ros节点) 发布的位置信息，从mocap坐标系转换至NED坐标系
 *      2. 订阅飞控发布的位置、速度及欧拉角信息，作对比用
 *      3. 存储飞行数据，实验分析及作图使用
 *      4. 选择激Mocap设备作为位置来源，发布位置及偏航角(xyz+yaw)给飞控
 *
***************************************************************************************************************************/

//头文件
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include "state_from_mavros.h"
#include "math_utils.h"
#include "control_utils.h"

using namespace std;

#define TRA_WINDOW 1000
#define TIMEOUT_MAX 0.05

//---------------------------------------相关参数-----------------------------------------------
int input_source;

string LIO_topic_, T265_topic_, Gazebo_topic_, VIO_topic_;

string object_name;
std::string subject_name;
std::string segment_name;
std::string child_frame_id;
std::string frame_id;

geometry_msgs::PoseStamped vision_pose_;
nav_msgs::Odometry odom_out_;
easondrone_msgs::DroneState Drone_State;
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;

//---------------------------------------发布相关变量--------------------------------------------
ros::Subscriber VICON_sub_, T265_sub_, Gazebo_sub_, optitrack_sub, LIO_sub_, VIO_sub_;
ros::Publisher vision_pose_pub_, odom_out_pub_, drone_state_pub, trajectory_pub;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void VICON_cb(const geometry_msgs::TransformStamped::ConstPtr& msg){
    if (input_source != 6){
        return;
    }

    vision_pose_.pose.position.x = msg->transform.translation.x;
    vision_pose_.pose.position.y = msg->transform.translation.y;
    vision_pose_.pose.position.z = msg->transform.translation.z;
    vision_pose_.pose.orientation.w = msg->transform.rotation.w;
    vision_pose_.pose.orientation.x = msg->transform.rotation.x;
    vision_pose_.pose.orientation.y = msg->transform.rotation.y;
    vision_pose_.pose.orientation.z = msg->transform.rotation.z;
}

void Gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg){
    if (input_source != 2){
        return;
    }

    vision_pose_.pose.position.x = msg->pose.pose.position.x;
    vision_pose_.pose.position.y = msg->pose.pose.position.y;
    vision_pose_.pose.position.z = msg->pose.pose.position.z;
    vision_pose_.pose.orientation.x = msg->pose.pose.orientation.x;
    vision_pose_.pose.orientation.y = msg->pose.pose.orientation.y;
    vision_pose_.pose.orientation.z = msg->pose.pose.orientation.z;
    vision_pose_.pose.orientation.w = msg->pose.pose.orientation.w;
}

void T265_cb(const nav_msgs::Odometry::ConstPtr &msg){
    if (input_source != 3){
        return;
    }

    odom_out_ = *msg;

    vision_pose_.pose.position.x = msg->pose.pose.position.x;
    vision_pose_.pose.position.y = msg->pose.pose.position.y;
    vision_pose_.pose.position.z = msg->pose.pose.position.z;
    vision_pose_.pose.orientation.x = msg->pose.pose.orientation.x;
    vision_pose_.pose.orientation.y = msg->pose.pose.orientation.y;
    vision_pose_.pose.orientation.z = msg->pose.pose.orientation.z;
    vision_pose_.pose.orientation.w = msg->pose.pose.orientation.w;
}

void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    if (input_source != 0){
        return;
    }

    vision_pose_.pose.position.x = msg->pose.position.x;
    vision_pose_.pose.position.y = msg->pose.position.y;
    vision_pose_.pose.position.z = msg->pose.position.z;
    vision_pose_.pose.orientation.x = msg->pose.orientation.x;
    vision_pose_.pose.orientation.y = msg->pose.orientation.y;
    vision_pose_.pose.orientation.z = msg->pose.orientation.z;
    vision_pose_.pose.orientation.w = msg->pose.orientation.w;
}

void LIO_cb(const nav_msgs::Odometry::ConstPtr &msg){
    if (input_source != 5){
        return;
    }

    odom_out_ = *msg;

    vision_pose_.pose.position.x = msg->pose.pose.position.x;
    vision_pose_.pose.position.y = msg->pose.pose.position.y;
    vision_pose_.pose.position.z = msg->pose.pose.position.z;
    vision_pose_.pose.orientation.x = msg->pose.pose.orientation.x;
    vision_pose_.pose.orientation.y = msg->pose.pose.orientation.y;
    vision_pose_.pose.orientation.z = msg->pose.pose.orientation.z;
    vision_pose_.pose.orientation.w = msg->pose.pose.orientation.w;
}

void VIO_cb(const nav_msgs::Odometry::ConstPtr &msg){
    if (input_source != 1){
        return;
    }

    odom_out_ = *msg;

    vision_pose_.pose.position.x = msg->pose.pose.position.x;
    vision_pose_.pose.position.y = msg->pose.pose.position.y;
    vision_pose_.pose.position.z = msg->pose.pose.position.z;
    vision_pose_.pose.orientation.x = msg->pose.pose.orientation.x;
    vision_pose_.pose.orientation.y = msg->pose.pose.orientation.y;
    vision_pose_.pose.orientation.z = msg->pose.pose.orientation.z;
    vision_pose_.pose.orientation.w = msg->pose.pose.orientation.w;
}

#endif //PX4CTRL_PX4_VISION_H
