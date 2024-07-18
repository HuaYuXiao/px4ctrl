#ifndef PX4CTRL_PX4_VISION_H
#define PX4CTRL_PX4_VISION_H

/***************************************************************************************************************************
 * px4_pos_estimator.cpp
 *
 * Author: Qyp
* Maintainer: Eason Hua
* Update Time: 2024.07.18
 *
 * 说明: mavros位置估计程序
 *      1. 订阅激光SLAM (cartorgrapher_ros节点) 发布的位置信息,从laser坐标系转换至NED坐标系
 *      2. 订阅Mocap设备 (vrpn-client-ros节点) 发布的位置信息，从mocap坐标系转换至NED坐标系
 *      3. 订阅飞控发布的位置、速度及欧拉角信息，作对比用
 *      4. 存储飞行数据，实验分析及作图使用
 *      5. 选择激光SLAM或者Mocap设备作为位置来源，发布位置及偏航角(xyz+yaw)给飞控
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
float rate_hz_;

int input_source;

Eigen::Vector3f pos_offset;
float yaw_offset;
float pitch_offset;
float roll_offset;

string object_name;
std::string subject_name;
std::string segment_name;
std::string child_frame_id;
std::string frame_id;
ros::Time last_timestamp;

// optitrack定位相关------------------------------------------
Eigen::Vector3d pos_drone_mocap; //无人机当前位置 (optitrack)
Eigen::Quaterniond q_mocap;
Eigen::Vector3d Euler_mocap; //无人机当前姿态 (optitrack)
// -------- vicon定位相关 --------
//无人机当前位置 (vicon)
Eigen::Vector3d pos_drone_vicon;
Eigen::Quaterniond q_vicon;
//无人机当前姿态 (vicon)
Eigen::Vector3d Euler_vicon;
//---------------------------------------laser定位相关------------------------------------------
Eigen::Vector3d pos_drone_laser; //无人机当前位置 (laser)
Eigen::Quaterniond q_laser;
Eigen::Vector3d Euler_laser; //无人机当前姿态(laser)
geometry_msgs::TransformStamped laser; //当前时刻cartorgrapher发布的数据
//---------------------------------------T265------------------------------------------
Eigen::Vector3d pos_drone_t265;
Eigen::Quaterniond q_t265;
Eigen::Vector3d Euler_t265;
//---------------------------------------gazebo真值相关------------------------------------------
Eigen::Vector3d pos_drone_gazebo;
Eigen::Quaterniond q_gazebo;
Eigen::Vector3d Euler_gazebo;
//---------------------------------------SLAM相关------------------------------------------
Eigen::Vector3d pos_drone_slam;
Eigen::Quaterniond q_slam;
Eigen::Vector3d Euler_slam;

Eigen::Vector3d pos_LIO;
Eigen::Quaterniond q_LIO;
Eigen::Vector3d Euler_LIO;

//---------------------------------------发布相关变量--------------------------------------------
ros::Subscriber vicon_sub, t265_sub, gazebo_sub, slam_sub, laser_sub, optitrack_sub, LIO_sub_;
ros::Publisher vision_pub, drone_state_pub, trajectory_pub;

easondrone_msgs::DroneState Drone_State;
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void vicon_cb(const geometry_msgs::TransformStamped::ConstPtr& msg){
    pos_drone_vicon = Eigen::Vector3d(msg->transform.translation.x - pos_offset[0],
                                      msg->transform.translation.y - pos_offset[1],
                                      msg->transform.translation.z - pos_offset[2]);

    q_vicon = Eigen::Quaterniond(msg->transform.rotation.w,
                                 msg->transform.rotation.x,
                                 msg->transform.rotation.y,
                                 msg->transform.rotation.z);

    Euler_vicon = quaternion_to_euler(q_vicon);
    // 将偏移量添加到欧拉角中
    Euler_vicon[0] -= yaw_offset;    // 偏航
    Euler_vicon[1] -= pitch_offset;  // 俯仰
    Euler_vicon[2] -= roll_offset;   // 横滚
}

void gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg){
    if (msg->header.frame_id == "map"){
        pos_drone_gazebo = Eigen::Vector3d(msg->pose.pose.position.x,
                                           msg->pose.pose.position.y,
                                           msg->pose.pose.position.z);
        q_gazebo = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                      msg->pose.pose.orientation.x,
                                      msg->pose.pose.orientation.y,
                                      msg->pose.pose.orientation.z);
        Euler_gazebo = quaternion_to_euler(q_gazebo);
        // Euler_gazebo[2] = Euler_gazebo[2] + yaw_offset;
        // q_gazebo = quaternion_from_rpy(Euler_gazebo);
    }
    else{
        cout << "[estimator] wrong gazebo ground truth frame id" << endl;
    }
}

void t265_cb(const nav_msgs::Odometry::ConstPtr &msg){
    if (msg->header.frame_id == "map"){
        pos_drone_t265 = Eigen::Vector3d(msg->pose.pose.position.x,
                                         msg->pose.pose.position.y,
                                         msg->pose.pose.position.z);
        // pos_drone_t265[0] = msg->pose.pose.position.x + pos_offset[0];
        // pos_drone_t265[1] = msg->pose.pose.position.y + pos_offset[1];
        // pos_drone_t265[2] = msg->pose.pose.position.z + pos_offset[2];

        q_t265 = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                    msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y,
                                    msg->pose.pose.orientation.z);
        Euler_t265 = quaternion_to_euler(q_gazebo);
        // Euler_t265[2] = Euler_t265[2] + yaw_offset;
        // q_t265 = quaternion_from_rpy(Euler_t265);
    }
    else{
        cout << "[estimator] wrong t265 frame id" << endl;
    }
}

void laser_cb(const tf2_msgs::TFMessage::ConstPtr &msg){
    //确定是cartographer发出来的/tf信息
    //有的时候/tf这个消息的发布者不止一个
    //可改成ＴＦ监听
    if (msg->transforms[0].header.frame_id == "map" &&
        msg->transforms[0].child_frame_id == "base_link" &&
        input_source == 1){
        laser = msg->transforms[0];

        //位置 xy  [将解算的位置从map坐标系转换至world坐标系]
        pos_drone_laser[0] = laser.transform.translation.x + pos_offset[0];
        pos_drone_laser[1] = laser.transform.translation.y + pos_offset[1];
        pos_drone_laser[2] = laser.transform.translation.z + pos_offset[2];

        // Read the Quaternion from the Carto Package [Frame: Laser[ENU]]
        Eigen::Quaterniond q_laser_enu(laser.transform.rotation.w, laser.transform.rotation.x, laser.transform.rotation.y, laser.transform.rotation.z);

        q_laser = q_laser_enu;

        // Transform the Quaternion to Euler Angles
        Euler_laser = quaternion_to_euler(q_laser);

        //cout << "Position [X Y Z] : " << pos_drone_laser[0] << " [ m ] "<< pos_drone_laser[1]<<" [ m ] "<< pos_drone_laser[2]<<" [ m ] "<<endl;
        //cout << "Euler [X Y Z] : " << Euler_laser[0] << " [m/s] "<< Euler_laser[1]<<" [m/s] "<< Euler_laser[2]<<" [m/s] "<<endl;
    }
}

void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    //位置 -- optitrack系 到 ENU系
    //Frame convention 0: Z-up -- 1: Y-up (See the configuration in the motive software)
    int optitrack_frame = 0;
    if (optitrack_frame == 0){
        // Read the Drone Position from the Vrpn Package [Frame: Vicon]  (Vicon to ENU frame)
        pos_drone_mocap = Eigen::Vector3d(msg->pose.position.x - pos_offset[0],
                                          msg->pose.position.y - pos_offset[1],
                                          msg->pose.position.z - pos_offset[2]);

//        pos_drone_mocap[0] = pos_drone_mocap[0];
//        pos_drone_mocap[1] = pos_drone_mocap[1];
//        pos_drone_mocap[2] = pos_drone_mocap[2];
        // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
        q_mocap = Eigen::Quaterniond(msg->pose.orientation.w,
                                     msg->pose.orientation.x,
                                     msg->pose.orientation.y,
                                     msg->pose.orientation.z);
    }
    else{
        // Read the Drone Position from the Vrpn Package [Frame: Vicon]  (Vicon to ENU frame)
        pos_drone_mocap = Eigen::Vector3d(-msg->pose.position.x,
                                          msg->pose.position.z,
                                          msg->pose.position.y);
        // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
        q_mocap = Eigen::Quaterniond(msg->pose.orientation.w,
                                     msg->pose.orientation.x,
                                     msg->pose.orientation.z,
                                     msg->pose.orientation.y); //Y-up convention, switch the q2 & q3
        pos_drone_mocap[0] = pos_drone_mocap[0] - pos_offset[0];
        pos_drone_mocap[1] = pos_drone_mocap[1] - pos_offset[1];
        pos_drone_mocap[2] = pos_drone_mocap[2] - pos_offset[2];
    }

    // Transform the Quaternion to Euler Angles
    Euler_mocap = quaternion_to_euler(q_mocap);

    last_timestamp = msg->header.stamp;
}

void slam_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    if (msg->header.frame_id == "map_slam"){
        pos_drone_slam = Eigen::Vector3d(msg->pose.position.x,
                                         msg->pose.position.y,
                                         msg->pose.position.z);
        // pos_drone_gazebo[0] = msg->pose.pose.position.x + pos_offset[0];
        // pos_drone_gazebo[1] = msg->pose.pose.position.y + pos_offset[1];
        // pos_drone_gazebo[2] = msg->pose.pose.position.z + pos_offset[2];

        q_slam = Eigen::Quaterniond(msg->pose.orientation.w,
                                    msg->pose.orientation.x,
                                    msg->pose.orientation.y,
                                    msg->pose.orientation.z);
        Euler_slam = quaternion_to_euler(q_slam);
        // Euler_gazebo[2] = Euler_gazebo[2] + yaw_offset;
        // q_gazebo = quaternion_from_rpy(Euler_gazebo);
    }else{
        cout << "[estimator] wrong slam frame id" << endl;
    }
}

void LIO_cb(const nav_msgs::Odometry::ConstPtr &msg){
    pos_LIO = Eigen::Vector3d(msg->pose.pose.position.x,
                                       msg->pose.pose.position.y,
                                       msg->pose.pose.position.z);
    q_LIO = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                  msg->pose.pose.orientation.x,
                                  msg->pose.pose.orientation.y,
                                  msg->pose.pose.orientation.z);
    Euler_LIO = quaternion_to_euler(q_LIO);
}

#endif //PX4CTRL_PX4_VISION_H
