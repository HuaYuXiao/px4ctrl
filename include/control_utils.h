/***************************************************************************************************************************
* control_utils.h
*
* Author: Qyp
* Maintainer: Eason Hua
* Update Time: 2024.07.04
***************************************************************************************************************************/
#ifndef CONTORL_UTILS_H
#define CONTORL_UTILS_H

#include <Eigen/Eigen>
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <math_utils.h>
#include <command_to_mavros.h>
#include <easondrone_msgs/Message.h>
#include <easondrone_msgs/ControlCommand.h>
#include <easondrone_msgs/DroneState.h>
#include <easondrone_msgs/PositionReference.h>
#include <easondrone_msgs/AttitudeReference.h>
#include <easondrone_msgs/ControlOutput.h>
#include "station_utils.h"

using namespace std;

#define NUM_POINT 2
#define NUM_MOTOR 4

#define MOTOR_P1 -0.00069
#define MOTOR_P2 0.01271
#define MOTOR_P3 -0.07948
#define MOTOR_P4 0.3052
#define MOTOR_P5 0.008775

#define thrust_max_single_motor 6.0

namespace control_utils{
// 【坐标系旋转函数】- 机体系到enu系
// body_frame是机体系,enu_frame是惯性系，yaw_angle是当前偏航角[rad]
void rotation_yaw(float yaw_angle, float body_frame[2], float enu_frame[2]){
    enu_frame[0] = body_frame[0] * cos(yaw_angle) - body_frame[1] * sin(yaw_angle);
    enu_frame[1] = body_frame[0] * sin(yaw_angle) + body_frame[1] * cos(yaw_angle);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 控 制 辅 助 函 数 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 
//计算位置误差
Eigen::Vector3f cal_pos_error(const easondrone_msgs::DroneState& _DroneState, const easondrone_msgs::PositionReference& _Reference_State){
    Eigen::Vector3f pos_error;

    for (int i=0; i<3; i++){
        pos_error[i] = _Reference_State.position_ref[i] - _DroneState.position[i];
    }

    return pos_error;
}

//计算速度误差
Eigen::Vector3f cal_vel_error(const easondrone_msgs::DroneState& _DroneState, const easondrone_msgs::PositionReference& _Reference_State){
    Eigen::Vector3f vel_error;

    for (int i=0; i<3; i++)
    {
        vel_error[i] = _Reference_State.velocity_ref[i] - _DroneState.velocity[i];
    }

    return vel_error;
}

Eigen::Vector3d accelToThrust(const Eigen::Vector3d& accel_sp, float mass, float tilt_max){
    Eigen::Vector3d thrust_sp;

    //除以电机个数得到单个电机的期望推力
    thrust_sp = mass * accel_sp / NUM_MOTOR;

    // 推力限幅，根据最大倾斜角及最大油门
    float thrust_max_XY_tilt = fabs(thrust_sp[2]) * tanf(tilt_max/180.0*M_PI);
    float thrust_max_XY = sqrtf(thrust_max_single_motor * thrust_max_single_motor - pow(thrust_sp[2],2));
    thrust_max_XY = min(thrust_max_XY_tilt, thrust_max_XY);

    if ((pow(thrust_sp[0],2) + pow(thrust_sp[1],2)) > pow(thrust_max_XY,2)){
        float mag = sqrtf((pow(thrust_sp[0],2) + pow(thrust_sp[1],2)));
        thrust_sp[0] = thrust_sp[0] / mag * thrust_max_XY;
        thrust_sp[1] = thrust_sp[1] / mag * thrust_max_XY;
    }
    
    return thrust_sp;   
}

Eigen::Vector3d thrustToThrottle(const Eigen::Vector3d& thrust_sp){
    Eigen::Vector3d throttle_sp;

    //电机模型，可通过辨识得到，推力-油门曲线
    for (int i=0; i<3; i++){
        throttle_sp[i] = MOTOR_P1 * pow(thrust_sp[i],4) + MOTOR_P2 * pow(thrust_sp[i],3) + MOTOR_P3 * pow(thrust_sp[i],2) + MOTOR_P4 * thrust_sp[i] + MOTOR_P5;
        // PX4内部默认假设 0.5油门为悬停推力 ， 在无人机重量为1kg时，直接除20得到0.5
        // throttle_sp[i] = thrust_sp[i]/20；
    }
    return throttle_sp; 
}

//Throttle to Attitude
//Thrust to Attitude
//Input: desired thrust (desired throttle [0,1]) and yaw_sp(rad)
//Output: desired attitude (quaternion)
easondrone_msgs::AttitudeReference ThrottleToAttitude(const Eigen::Vector3d& thr_sp, float yaw_sp)
{
    easondrone_msgs::AttitudeReference _AttitudeReference;
    Eigen::Vector3d att_sp;
    att_sp[2] = yaw_sp;

    // desired body_z axis = -normalize(thrust_vector)
    Eigen::Vector3d body_x, body_y, body_z;

    double thr_sp_length = thr_sp.norm();

    //cout << "thr_sp_length : "<< thr_sp_length << endl;

    if (thr_sp_length > 0.00001f) {
            body_z = thr_sp.normalized();

    }
    else {
            // no thrust, set Z axis to safe value
            body_z = Eigen::Vector3d(0.0f, 0.0f, 1.0f);
    }

    // vector of desired yaw direction in XY plane, rotated by PI/2
    Eigen::Vector3d y_C = Eigen::Vector3d(-sinf(yaw_sp),cosf(yaw_sp),0.0);

    if (fabsf(body_z(2)) > 0.000001f) {
            // desired body_x axis, orthogonal to body_z
            body_x = y_C.cross(body_z);

            // keep nose to front while inverted upside down
            if (body_z(2) < 0.0f) {
                    body_x = -body_x;
            }

            body_x.normalize();

    }
    else {
            // desired thrust is in XY plane, set X downside to construct correct matrix,
            // but yaw component will not be used actually
            body_x = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
            body_x(2) = 1.0f;
    }

    // desired body_y axis
    body_y = body_z.cross(body_x);

    Eigen::Matrix3d R_sp;

    // fill rotation matrix
    for (int i = 0; i < 3; i++) {
            R_sp(i, 0) = body_x(i);
            R_sp(i, 1) = body_y(i);
            R_sp(i, 2) = body_z(i);
    }

    Eigen::Quaterniond q_sp(R_sp);

    rotation_to_euler(R_sp, att_sp);

    //cout << "Desired euler [R P Y]: "<< att_sp[0]* 180/M_PI <<" [deg] " << att_sp[1]* 180/M_PI <<" [deg] "<< att_sp[2]* 180/M_PI <<" [deg] "<< endl;
    //cout << "Desired Thrust: "<< thr_sp_length<< endl;
//    cout << "q_sp [x y z w]: "<< q_sp.x() <<" [ ] " << q_sp.y() <<" [ ] "<<q_sp.z() <<" [ ] "<<q_sp.w() <<" [ ] "<<endl;
//    cout << "R_sp : "<< R_sp(0, 0) <<" " << R_sp(0, 1) <<" "<< R_sp(0, 2) << endl;
//    cout << "     : "<< R_sp(1, 0) <<" " << R_sp(1, 1) <<" "<< R_sp(1, 2) << endl;
//    cout << "     : "<< R_sp(2, 0) <<" " << R_sp(2, 1) <<" "<< R_sp(2, 2) << endl;


    _AttitudeReference.throttle_sp[0] = thr_sp[0];
    _AttitudeReference.throttle_sp[1] = thr_sp[1];
    _AttitudeReference.throttle_sp[2] = thr_sp[2];

    //期望油门
    _AttitudeReference.desired_throttle = thr_sp_length; 

    _AttitudeReference.desired_att_q.w = q_sp.w();
    _AttitudeReference.desired_att_q.x = q_sp.x();
    _AttitudeReference.desired_att_q.y = q_sp.y();
    _AttitudeReference.desired_att_q.z = q_sp.z();

    _AttitudeReference.desired_attitude[0] = att_sp[0];  
    _AttitudeReference.desired_attitude[1] = att_sp[1]; 
    _AttitudeReference.desired_attitude[2] = att_sp[2]; 

    return _AttitudeReference;
}
}
#endif
