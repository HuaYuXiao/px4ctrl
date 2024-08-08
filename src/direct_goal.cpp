//
// Created by hyx020222 on 7/26/24.
// Last modified on 2024.08.08
//

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <easondrone_msgs/ControlCommand.h>

easondrone_msgs::ControlCommand ctrl_cmd;                      //无人机当前执行命令

ros::Subscriber goal_sub_, odom_sub_;
ros::Publisher easondrone_ctrl_pub_;

void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ctrl_cmd.poscmd.position.x = msg->pose.position.x;
    ctrl_cmd.poscmd.position.y = msg->pose.position.y;
    ctrl_cmd.poscmd.yaw = 2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);

    ctrl_cmd.header.stamp = ros::Time::now();
    easondrone_ctrl_pub_.publish(ctrl_cmd);
}

// 保存无人机当前里程计信息，包括位置、速度和姿态
void odometryCallback(const nav_msgs::OdometryConstPtr &msg){
    ctrl_cmd.poscmd.position.z = msg->pose.pose.position.z;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "direct_goal");
    ros::NodeHandle nh;

    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>
            ("/move_base_simple/goal", 10, goal_cb);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>
            ("/mavros/local_position/odom", 10, odometryCallback);

    //【订阅】指令 本话题为任务模块生成的控制指令
    easondrone_ctrl_pub_ = nh.advertise<easondrone_msgs::ControlCommand>
            ("/easondrone/control_command", 10);

    // 初始化命令
    ctrl_cmd.frame = easondrone_msgs::ControlCommand::ENU;

    ros::spin();
}
