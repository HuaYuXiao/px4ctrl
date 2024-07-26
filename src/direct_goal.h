//
// Created by hyx020222 on 7/26/24.
//

#ifndef PX4CTRL_DIRECT_GOAL_H
#define PX4CTRL_DIRECT_GOAL_H

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <easondrone_msgs/ControlCommand.h>


easondrone_msgs::ControlCommand easondrone_ctrl_cmd_;                      //无人机当前执行命令

ros::Subscriber goal_sub_, odom_sub_;
ros::Publisher easondrone_ctrl_cmd_pub_;

#endif //PX4CTRL_DIRECT_GOAL_H
