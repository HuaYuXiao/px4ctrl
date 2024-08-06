//
// Created by hyx020222 on 7/9/24.
// last updated on 2024.08.06
//

#ifndef EASONDRONE_CONTROL_PX4CTRL_H
#define EASONDRONE_CONTROL_PX4CTRL_H

#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include "control_utils.h"
#include "Position_Controller/pos_controller_cascade_PID.h"

using namespace std;


#define TRA_WINDOW 1000
#define Takeoff_height_ 1.5                                       //默认起飞高度

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float cur_time;                                             //程序运行时间
float dt;

Eigen::Vector3d Takeoff_position;                              // 起飞位置
easondrone_msgs::DroneState _DroneState;                          //无人机状态量
mavros_msgs::State mavros_state;

bool have_odom_;
Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
Eigen::Quaterniond odom_orient_;
double odom_yaw_;

//变量声明 - 服务
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandBool arm_cmd;

easondrone_msgs::ControlCommand Command_Now;                      //无人机当前执行命令
easondrone_msgs::ControlCommand Command_Last;                     //无人机上一条执行命令
easondrone_msgs::ControlOutput _ControlOutput;
easondrone_msgs::AttitudeReference _AttitudeReference;           //位置控制器输出，即姿态环参考量

Eigen::Vector3d throttle_sp;

ros::Subscriber easondrone_ctrl_sub_, station_command_sub, drone_state_sub, mavros_state_sub_, odom_sub_;
ros::Publisher att_ref_pub, setpoint_raw_attitude_pub_;
ros::ServiceClient set_mode_client_, arming_client_;


//【Body_to_ENU】 机体系移动。
void Body_to_ENU(){
    if(Command_Now.Reference_State.Move_mode  & 0b00){
        // XYZ_POS
        float d_pos_body[2] = {Command_Now.Reference_State.position_ref[0], Command_Now.Reference_State.position_ref[1]};         //the desired xy position in Body Frame
        float d_pos_enu[2];                                                           //the desired xy position in enu Frame (The origin point is the drone)
        control_utils::rotation_yaw(_DroneState.attitude[2], d_pos_body, d_pos_enu);
        Command_Now.Reference_State.position_ref[0] = odom_pos_[0] + d_pos_enu[0];
        Command_Now.Reference_State.position_ref[1] = odom_pos_[1] + d_pos_enu[1];

        Command_Now.Reference_State.velocity_ref[0] = 0;
        Command_Now.Reference_State.velocity_ref[1] = 0;
    }
    else if( Command_Now.Reference_State.Move_mode  & 0b01){
//        TODO: XY_POS_Z_VEL
//        float d_vel_body[2] = {Command_Now.Reference_State.velocity_ref[0], Command_Now.Reference_State.velocity_ref[1]};         //the desired xy velocity in Body Frame
//        float d_vel_enu[2];                                                           //the desired xy velocity in NED Frame
//        //根据无人机当前偏航角进行坐标系转换
//        control_utils::rotation_yaw(_DroneState.attitude[2], d_vel_body, d_vel_enu);
//
//        Command_Now.Reference_State.position_ref[0] = odom_pos_[0] + d_pos_enu[0];
//        Command_Now.Reference_State.position_ref[1] = odom_pos_[1] + d_pos_enu[1];
//        Command_Now.Reference_State.velocity_ref[0] = 0;
//        Command_Now.Reference_State.velocity_ref[1] = 0;
    }
    else if( Command_Now.Reference_State.Move_mode  & 0b10){
        // XY_VEL_Z_POS
        Command_Now.Reference_State.position_ref[0] = 0;
        Command_Now.Reference_State.position_ref[1] = 0;

        float d_vel_body[2] = {Command_Now.Reference_State.velocity_ref[0], Command_Now.Reference_State.velocity_ref[1]};         //the desired xy velocity in Body Frame
        float d_vel_enu[2];                                                           //the desired xy velocity in NED Frame
        //根据无人机当前偏航角进行坐标系转换
        control_utils::rotation_yaw(_DroneState.attitude[2], d_vel_body, d_vel_enu);
        Command_Now.Reference_State.velocity_ref[0] = d_vel_enu[0];
        Command_Now.Reference_State.velocity_ref[1] = d_vel_enu[1];
    }
    else if(Command_Now.Reference_State.Move_mode & 0b11){
        // XYZ_VEL
        float d_vel_body[2] = {Command_Now.Reference_State.velocity_ref[0], Command_Now.Reference_State.velocity_ref[1]};
        float d_vel_enu[2];                                                           //the desired xy velocity in NED Frame
        //根据无人机当前偏航角进行坐标系转换
        control_utils::rotation_yaw(_DroneState.attitude[2], d_vel_body, d_vel_enu);
        Command_Now.Reference_State.velocity_ref[0] = d_vel_enu[0];
        Command_Now.Reference_State.velocity_ref[1] = d_vel_enu[1];
        Command_Now.Reference_State.velocity_ref[2] = Command_Now.Reference_State.velocity_ref[2];
    }
    else if(Command_Now.Reference_State.Move_mode & 0b110){
//POS_VEL_ACC
        cout << "[control] POS_VEL_ACC" << endl;
        float d_pos_body[2] = {Command_Now.Reference_State.position_ref[0], Command_Now.Reference_State.position_ref[1]};         //the desired xy position in Body Frame
        float d_pos_enu[2];                                                           //the desired xy position in enu Frame (The origin point is the drone)
        control_utils::rotation_yaw(_DroneState.attitude[2], d_pos_body, d_pos_enu);
        Command_Now.Reference_State.position_ref[0] = odom_pos_[0] + d_pos_enu[0];
        Command_Now.Reference_State.position_ref[1] = odom_pos_[1] + d_pos_enu[1];
        Command_Now.Reference_State.position_ref[2] = Command_Now.Reference_State.position_ref[2];

        float d_vel_body[2] = {Command_Now.Reference_State.velocity_ref[0], Command_Now.Reference_State.velocity_ref[1]};         //the desired xy velocity in Body Frame
        float d_vel_enu[2];                                                           //the desired xy velocity in NED Frame
        //根据无人机当前偏航角进行坐标系转换
        control_utils::rotation_yaw(_DroneState.attitude[2], d_vel_body, d_vel_enu);
        Command_Now.Reference_State.velocity_ref[0] = d_vel_enu[0];
        Command_Now.Reference_State.velocity_ref[1] = d_vel_enu[1];
        Command_Now.Reference_State.velocity_ref[2] = Command_Now.Reference_State.velocity_ref[2];

        float d_acc_body[2] = {Command_Now.Reference_State.acceleration_ref[0], Command_Now.Reference_State.acceleration_ref[1]};         //the desired xy acceleration in Body Frame
        float d_acc_enu[2];                                                           //the desired xy acceleration in NED Frame
        control_utils::rotation_yaw(_DroneState.attitude[2], d_acc_body, d_acc_enu);
        Command_Now.Reference_State.acceleration_ref[0] = d_acc_enu[0];
        Command_Now.Reference_State.acceleration_ref[1] = d_acc_enu[1];
        Command_Now.Reference_State.acceleration_ref[2] = Command_Now.Reference_State.acceleration_ref[2];
    }
    else{
        cout << "[px4ctrl] unsupported Move_mode: " << Command_Now.Reference_State.Move_mode << endl;

        return;
    }

    Command_Now.Reference_State.yaw_ref = _DroneState.attitude[2] + Command_Now.Reference_State.yaw_ref;

    float d_acc_body[2] = {Command_Now.Reference_State.acceleration_ref[0], Command_Now.Reference_State.acceleration_ref[1]};
    float d_acc_enu[2];

    control_utils::rotation_yaw(_DroneState.attitude[2], d_acc_body, d_acc_enu);
    Command_Now.Reference_State.acceleration_ref[0] = d_acc_enu[0];
    Command_Now.Reference_State.acceleration_ref[1] = d_acc_enu[1];
    Command_Now.Reference_State.acceleration_ref[2] = Command_Now.Reference_State.acceleration_ref[2];
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void easondrone_ctrl_cb_(const easondrone_msgs::ControlCommand::ConstPtr& msg){
    Command_Now = *msg;

    // 无人机一旦接受到Disarm指令，则会屏蔽其他指令
    if(Command_Last.Mode == easondrone_msgs::ControlCommand::Disarm){
        Command_Now = Command_Last;
    }
}

void drone_state_cb(const easondrone_msgs::DroneState::ConstPtr& msg){
    _DroneState = *msg;
}

void mavros_state_cb(const mavros_msgs::State::ConstPtr &msg){
    mavros_state = *msg;
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
