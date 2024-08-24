/*
 *
 * */

#ifndef PX4CTRL_NODE_H
#define PX4CTRL_NODE_H

#include <Eigen/Eigen>
#include <sstream>
#include <string>
#include <boost/math/special_functions.hpp>

#include <ros/ros.h>
#include <tf/tf.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <easondrone_msgs/ControlCommand.h>

using namespace std;

namespace PX4CtrlFSM{
    const double POS_ACCEPT = 0.2;
    const double YAW_ACCEPT = 15 / 180.0 * M_PI;

    mavros_msgs::State current_state;
    // odometry state
    bool have_odom_;
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;
    double odom_roll_, odom_pitch_, odom_yaw_;

    geographic_msgs::GeoPointStamped gp_origin;
    geometry_msgs::PoseStamped pose;
    mavros_msgs::PositionTarget pos_setpoint;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    //无人机当前执行命令
    easondrone_msgs::ControlCommand ctrl_cmd;

    ros::Timer gp_origin_timer_;
    ros::Subscriber state_sub, easondrone_ctrl_sub_, odom_sub_;
    ros::Publisher gp_origin_pub, local_pos_pub, setpoint_raw_local_pub, setpoint_raw_global_pub, setpoint_raw_attitude_pub_;
    //变量声明 - 服务
    ros::ServiceClient arming_client, set_mode_client;

    /******** callback ********/
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
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

        tf::Quaternion odom_q_(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
        );

        tf::Matrix3x3(odom_q_).getRPY(odom_roll_, odom_pitch_, odom_yaw_);
    }

    void gpOriginCallback(const ros::TimerEvent &e){
        gp_origin.header.stamp = ros::Time::now();

        gp_origin_pub.publish(gp_origin);
    }

    void easondrone_ctrl_cb_(const easondrone_msgs::ControlCommand::ConstPtr& msg){
        ctrl_cmd = *msg;
    }
}

#endif //PX4CTRL_NODE_H
