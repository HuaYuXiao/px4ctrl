/*
 * Maintainer: Eason Hua
 * Update Time: 2024.09.09
 * 12010508@mail.sustech.edu.cn
 */

#ifndef PX4CTRL_NODE_H
#define PX4CTRL_NODE_H

#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <sstream>
#include <string>
#include <set>
#include <limits>
#include <boost/math/special_functions.hpp>

#include <ros/ros.h>
#include <tf/tf.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <easondrone_msgs/ControlCommand.h>

using namespace std;

#define ODOM_LOST 5.0
#define RC_CTRL 3.0
#define POS_ACCEPT 0.2
#define YAW_ACCEPT 15 / 180.0 * M_PI

namespace Utils{
    const string RESET_COLOR = "\033[0m";
    const string RED_COLOR = "\033[31m";
    const string GREEN_COLOR = "\033[32m";
    const string YELLOW_COLOR = "\033[33m";
    const string BLUE_COLOR = "\033[34m";
    const string MAGENTA_COLOR = "\033[35m";
    const string CYAN_COLOR = "\033[36m";
    const string WHITE_COLOR = "\033[37m";

    void cout_color(const string &msg, const string &color) {
        cout << color << msg << RESET_COLOR << endl;
    }

    void clear_cin(const string &msg){
        // Clear error flags
        cin.clear();
        // Discard invalid input
        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        cout_color(msg, RED_COLOR);
    }
}

namespace PX4CtrlFSM{
    ros::Subscriber state_sub;
    mavros_msgs::State current_state;

    // odometry state
    ros::Subscriber odom_sub_;
    bool have_odom_;
    // TODO: change to odom lost check
    ros::Time last_odom_stamp_;
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;
    double odom_roll_, odom_pitch_, odom_yaw_;

    ros::Subscriber VICON_sub_, T265_sub_, Gazebo_sub_, optitrack_sub, LIO_sub_, VIO_sub_;
    geometry_msgs::PoseStamped vision_pose_;
    nav_msgs::Odometry odom_out_;
    int ekf2_source_;
    string LIO_topic_, T265_topic_, Gazebo_topic_, VIO_topic_;
    string object_name;
    std::string subject_name;
    std::string segment_name;
    ros::Publisher vision_pose_pub_, odom_out_pub_;

    ros::Timer gp_origin_timer_;
    geographic_msgs::GeoPointStamped gp_origin;
    ros::Publisher gp_origin_pub;

    ros::Subscriber rc_sub_;
    ros::Time last_rc_stamp_;

    geometry_msgs::PoseStamped pose;
    mavros_msgs::PositionTarget pos_setpoint;
    // 发布相关变量
    ros::Publisher local_pos_pub, setpoint_raw_local_pub, setpoint_raw_global_pub, setpoint_raw_attitude_pub_;

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    //变量声明 - 服务
    ros::ServiceClient arming_client, set_mode_client;

    ros::Subscriber easondrone_ctrl_sub_;
    bool task_done_;
    //无人机当前执行命令
    easondrone_msgs::ControlCommand ctrl_cmd_in_, ctrl_cmd_out_;
    ros::Publisher easondrone_ctrl_pub_;

    /******** callback ********/
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }

    // 保存无人机当前里程计信息，包括位置、速度和姿态
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
        // TODO: add odom lost check
        have_odom_ = true;
        last_odom_stamp_ = ros::Time::now();

        odom_pos_ << msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z;

        odom_vel_ << msg->twist.twist.linear.x,
                msg->twist.twist.linear.y,
                msg->twist.twist.linear.z;

        //odom_acc_ = estimateAcc( msg );

        // 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
        // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        tf::Quaternion odom_q_(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
        );

        tf::Matrix3x3(odom_q_).getRPY(odom_roll_, odom_pitch_, odom_yaw_);
    }

    void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg){
        if (msg->channels[0] != 1500 || // 1050 left,  1950 right
            msg->channels[1] != 1500 || // 1050 front, 1950 back
            msg->channels[2] != 1500 || // 1050 down,  1950 upward
            msg->channels[3] != 1500 || // 1050 CCW,   1950 CW
            msg->channels[4] != 1050 || // 1050 POS,   1500 ALT,      1950 STA
            msg->channels[5] != 1500){  // 1500 Ready, 1950 Not Ready
            last_rc_stamp_ = ros::Time::now();
            
            // reject command from easondrone_msgs::ControlCommand
            task_done_ = true;
        }
        // do not set task_done_ to false! because it should be set to false only when easondrone_cmd is received
    }

    void gpOriginCallback(const ros::TimerEvent& e){
        gp_origin.header.stamp = ros::Time::now();

        gp_origin_pub.publish(gp_origin);
    }

    void easondrone_ctrl_cb_(const easondrone_msgs::ControlCommand::ConstPtr& msg){
        ctrl_cmd_in_ = *msg;

        task_done_ = false;
    }

    void VICON_cb(const geometry_msgs::TransformStamped::ConstPtr& msg){
        if (ekf2_source_ != 6){
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

    void Gazebo_cb(const nav_msgs::Odometry::ConstPtr& msg){
        if (ekf2_source_ != 2){
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

    void T265_cb(const nav_msgs::Odometry::ConstPtr& msg){
        if (ekf2_source_ != 3){
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

    void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        if (ekf2_source_ != 0){
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

    void LIO_cb(const nav_msgs::Odometry::ConstPtr& msg){
        if (ekf2_source_ != 5){
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

    void VIO_cb(const nav_msgs::Odometry::ConstPtr& msg){
        if (ekf2_source_ != 1){
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
}

#endif //PX4CTRL_NODE_H
