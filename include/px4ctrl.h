//
// Created by hyx020222 on 7/9/24.
//

#ifndef EASONDRONE_CONTROL_PX4CTRL_H
#define EASONDRONE_CONTROL_PX4CTRL_H

#include <ros/ros.h>

#include "state_from_mavros.h"
#include "command_to_mavros.h"
#include "control_utils.h"
#include "control_common.h"
#include "Position_Controller/pos_controller_cascade_PID.h"
#include "Position_Controller/pos_controller_PID.h"
#include "Position_Controller/pos_controller_UDE.h"
#include "Position_Controller/pos_controller_NE.h"
#include "Position_Controller/pos_controller_Passivity.h"

#define NODE_NAME "px4ctrl"

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float rate_hz_;

float cur_time;                                             //程序运行时间
string controller_type_;                                      //控制器类型
float Takeoff_height_;                                       //默认起飞高度
float Disarm_height_;                                        //自动上锁高度
float Land_speed_;                                           //降落速度

//Geigraphical fence 地理围栏
Eigen::Vector2f geo_fence_x;
Eigen::Vector2f geo_fence_y;
Eigen::Vector2f geo_fence_z;

Eigen::Vector3d Takeoff_position;                              // 起飞位置
easondrone_msgs::DroneState _DroneState;                          //无人机状态量
mavros_msgs::State mavros_state;

easondrone_msgs::ControlCommand Command_Now;                      //无人机当前执行命令
easondrone_msgs::ControlCommand Command_Last;                     //无人机上一条执行命令
easondrone_msgs::ControlOutput _ControlOutput;
easondrone_msgs::AttitudeReference _AttitudeReference;           //位置控制器输出，即姿态环参考量

float dt = 0.02;

ros::Publisher att_ref_pub;

Eigen::Vector3d throttle_sp;

bool check_safety(){
    if (_DroneState.position[0] <= geo_fence_x[0] ||
        _DroneState.position[0] >= geo_fence_x[1] ||
        _DroneState.position[1] <= geo_fence_y[0] ||
        _DroneState.position[1] >= geo_fence_y[1] ||
        _DroneState.position[2] <= geo_fence_z[0] ||
        _DroneState.position[2] >= geo_fence_z[1]){

        cout << "[control] Out of geo fence, the drone is landing" << endl;

        return false;
    }

    return true;
}

//【Body_to_ENU】 机体系移动。
void Body_to_ENU(){
    if(Command_Now.Reference_State.Move_mode  & 0b00){
        // XYZ_POS
        float d_pos_body[2] = {Command_Now.Reference_State.position_ref[0], Command_Now.Reference_State.position_ref[1]};         //the desired xy position in Body Frame
        float d_pos_enu[2];                                                           //the desired xy position in enu Frame (The origin point is the drone)
        control_utils::rotation_yaw(_DroneState.attitude[2], d_pos_body, d_pos_enu);
        Command_Now.Reference_State.position_ref[0] = _DroneState.position[0] + d_pos_enu[0];
        Command_Now.Reference_State.position_ref[1] = _DroneState.position[1] + d_pos_enu[1];

        Command_Now.Reference_State.velocity_ref[0] = 0;
        Command_Now.Reference_State.velocity_ref[1] = 0;
    }else if( Command_Now.Reference_State.Move_mode  & 0b01){
//        TODO: XY_POS_Z_VEL
//        float d_vel_body[2] = {Command_Now.Reference_State.velocity_ref[0], Command_Now.Reference_State.velocity_ref[1]};         //the desired xy velocity in Body Frame
//        float d_vel_enu[2];                                                           //the desired xy velocity in NED Frame
//        //根据无人机当前偏航角进行坐标系转换
//        control_utils::rotation_yaw(_DroneState.attitude[2], d_vel_body, d_vel_enu);
//
//        Command_Now.Reference_State.position_ref[0] = _DroneState.position[0] + d_pos_enu[0];
//        Command_Now.Reference_State.position_ref[1] = _DroneState.position[1] + d_pos_enu[1];
//        Command_Now.Reference_State.velocity_ref[0] = 0;
//        Command_Now.Reference_State.velocity_ref[1] = 0;
    }else if( Command_Now.Reference_State.Move_mode  & 0b10){
        // XY_VEL_Z_POS
        Command_Now.Reference_State.position_ref[0] = 0;
        Command_Now.Reference_State.position_ref[1] = 0;

        float d_vel_body[2] = {Command_Now.Reference_State.velocity_ref[0], Command_Now.Reference_State.velocity_ref[1]};         //the desired xy velocity in Body Frame
        float d_vel_enu[2];                                                           //the desired xy velocity in NED Frame
        //根据无人机当前偏航角进行坐标系转换
        control_utils::rotation_yaw(_DroneState.attitude[2], d_vel_body, d_vel_enu);
        Command_Now.Reference_State.velocity_ref[0] = d_vel_enu[0];
        Command_Now.Reference_State.velocity_ref[1] = d_vel_enu[1];
    }else if(Command_Now.Reference_State.Move_mode & 0b11){
        // XYZ_VEL
        float d_vel_body[2] = {Command_Now.Reference_State.velocity_ref[0], Command_Now.Reference_State.velocity_ref[1]};
        float d_vel_enu[2];                                                           //the desired xy velocity in NED Frame
        //根据无人机当前偏航角进行坐标系转换
        control_utils::rotation_yaw(_DroneState.attitude[2], d_vel_body, d_vel_enu);
        Command_Now.Reference_State.velocity_ref[0] = d_vel_enu[0];
        Command_Now.Reference_State.velocity_ref[1] = d_vel_enu[1];
        Command_Now.Reference_State.velocity_ref[2] = Command_Now.Reference_State.velocity_ref[2];
    }else if(Command_Now.Reference_State.Move_mode & 0b110){
//POS_VEL_ACC
        cout << "[control] POS_VEL_ACC" << endl;
        float d_pos_body[2] = {Command_Now.Reference_State.position_ref[0], Command_Now.Reference_State.position_ref[1]};         //the desired xy position in Body Frame
        float d_pos_enu[2];                                                           //the desired xy position in enu Frame (The origin point is the drone)
        control_utils::rotation_yaw(_DroneState.attitude[2], d_pos_body, d_pos_enu);
        Command_Now.Reference_State.position_ref[0] = _DroneState.position[0] + d_pos_enu[0];
        Command_Now.Reference_State.position_ref[1] = _DroneState.position[1] + d_pos_enu[1];
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
    }else{
        cout << "[control] unsupported Move_mode: " << Command_Now.Reference_State.Move_mode << endl;

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
void Command_cb(const easondrone_msgs::ControlCommand::ConstPtr& msg){
    // CommandID必须递增才会被记录
    if( msg->Command_ID  >  Command_Now.Command_ID ){
        Command_Now = *msg;
    }else{
        cout << "[control] Wrong Command ID" << endl;
        cout << Command_Now << endl;
    }

    // 无人机一旦接受到Disarm指令，则会屏蔽其他指令
    if(Command_Last.Mode == easondrone_msgs::ControlCommand::Disarm){
        Command_Now = Command_Last;
    }
}

void station_command_cb(const easondrone_msgs::ControlCommand::ConstPtr& msg){
    Command_Now = *msg;
    cout << "[control] Get a command from Station" << endl;

    // 无人机一旦接受到Disarm指令，则会屏蔽其他指令
    if(Command_Last.Mode == easondrone_msgs::ControlCommand::Disarm){
        Command_Now = Command_Last;
    }
}

void drone_state_cb(const easondrone_msgs::DroneState::ConstPtr& msg){
    _DroneState = *msg;

    _DroneState.time_from_start = cur_time;
}

#endif //EASONDRONE_CONTROL_PX4CTRL_H
