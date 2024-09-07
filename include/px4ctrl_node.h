/*
 * Maintainer: Eason Hua
 * Update Time: 2024.08.24
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
    mavros_msgs::State current_state;
    // odometry state
    // TODO: change to odom lost check
    bool have_odom_;
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;
    double odom_roll_, odom_pitch_, odom_yaw_;
    geometry_msgs::PoseStamped vision_pose_;

    int input_source;
    string LIO_topic_, T265_topic_, Gazebo_topic_, VIO_topic_;
    string object_name;
    std::string subject_name;
    std::string segment_name;

    bool task_done_;
    geographic_msgs::GeoPointStamped gp_origin;
    geometry_msgs::PoseStamped pose;
    mavros_msgs::PositionTarget pos_setpoint;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    //无人机当前执行命令
    easondrone_msgs::ControlCommand ctrl_cmd_in_, ctrl_cmd_out_;
    nav_msgs::Odometry odom_out_;
    std::vector<geometry_msgs::PoseStamped> posehistory_vector_;

    // 发布相关变量
    ros::Timer gp_origin_timer_;
    ros::Subscriber VICON_sub_, T265_sub_, Gazebo_sub_, optitrack_sub, LIO_sub_, VIO_sub_, \
                    state_sub, easondrone_ctrl_sub_, odom_sub_;
    ros::Publisher vision_pose_pub_, odom_out_pub_, trajectory_pub, \
                    gp_origin_pub, local_pos_pub, setpoint_raw_local_pub, setpoint_raw_global_pub, setpoint_raw_attitude_pub_;
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

        // 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
        // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        tf::Quaternion odom_q_(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
        );

        tf::Matrix3x3(odom_q_).getRPY(odom_roll_, odom_pitch_, odom_yaw_);

        // 发布无人机的轨迹 用作rviz中显示
//        geometry_msgs::PoseStamped posestamped_;
//        posestamped_.pose.position = msg->pose.pose.position;
//        posestamped_.pose.orientation = msg->pose.pose.orientation;
//
//        posehistory_vector_.insert(posehistory_vector_.begin(), posestamped_);
//        if (posehistory_vector_.size() > TRA_WINDOW){
//            posehistory_vector_.pop_back();
//        }
//
//        nav_msgs::Path drone_trajectory;
//        drone_trajectory.header = msg->header;
//        drone_trajectory.poses = posehistory_vector_;
//
//        trajectory_pub.publish(drone_trajectory);
    }

    void gpOriginCallback(const ros::TimerEvent &e){
        gp_origin.header.stamp = ros::Time::now();

        gp_origin_pub.publish(gp_origin);
    }

    void easondrone_ctrl_cb_(const easondrone_msgs::ControlCommand::ConstPtr& msg){
        ctrl_cmd_in_ = *msg;

        task_done_ = false;
    }

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
}

#endif //PX4CTRL_NODE_H
