//
// Created by hyx020222 on 7/26/24.
//

#include "direct_goal.h"

void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    easondrone_ctrl_cmd_.Reference_State.position_ref[0] = msg->pose.position.x;
    easondrone_ctrl_cmd_.Reference_State.position_ref[1] = msg->pose.position.y;
    easondrone_ctrl_cmd_.Reference_State.yaw_ref = 2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);

    easondrone_ctrl_cmd_.header.stamp = ros::Time::now();
    easondrone_ctrl_cmd_pub_.publish(easondrone_ctrl_cmd_);
}

// 保存无人机当前里程计信息，包括位置、速度和姿态
void odometryCallback(const nav_msgs::OdometryConstPtr &msg){
    easondrone_ctrl_cmd_.Reference_State.position_ref[2] = msg->pose.pose.position.z;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "direct_goal");
    ros::NodeHandle nh;

    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>
            ("/move_base_simple/goal", 10, goal_cb);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>
            ("/mavros/local_position/odom", 10, odometryCallback);

    //【订阅】指令 本话题为任务模块生成的控制指令
    easondrone_ctrl_cmd_pub_ = nh.advertise<easondrone_msgs::ControlCommand>
            ("/easondrone/control_command", 10);

    // 初始化命令- 默认设置：Idle模式 电机怠速旋转 等待来自上层的控制指令
    easondrone_ctrl_cmd_.Mode                                = easondrone_msgs::ControlCommand::Move;
    easondrone_ctrl_cmd_.Reference_State.Move_mode           = easondrone_msgs::PositionReference::XYZ_POS;
    easondrone_ctrl_cmd_.Reference_State.Move_frame          = easondrone_msgs::PositionReference::ENU_FRAME;

    ros::spin();
}
