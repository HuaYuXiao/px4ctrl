/*
 * Maintainer: Eason Hua
 * Update Time: 2024.09.07
 * 12010508@mail.sustech.edu.cn
 */

#include "px4ctrl_node.h"

using namespace PX4CtrlFSM;

/****** 主函数 ******/
int main(int argc, char **argv){
    ros::init(argc, argv, "ekf2_fusion");
    ros::NodeHandle nh("~");

    // 定位数据输入源
    nh.param<int>("ekf2_source", ekf2_source_, 2);

    switch (ekf2_source_){
        case 0:{
            cout << "[px4ctrl_vision] Using OptiTrack as input source." << endl;

            // 动作捕捉设备中设定的刚体名字
            nh.param<string>("object_name", object_name, "p450");

            // 【订阅】optitrack估计位置
            optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("/vrpn_client_node/"+ object_name + "/pose", 100, optitrack_cb);

            break;
        }

        case 1:{
            cout << "[px4ctrl_vision] Using VIO as input source." << endl;

            nh.param<string>("VIO_topic", VIO_topic_, "/vins_estimator/odometry");

            // subscribe to odometry from LIO
            VIO_sub_ = nh.subscribe<nav_msgs::Odometry>
                    (VIO_topic_, 100, VIO_cb);

            break;
        }

        case 2:{
            cout << "[px4ctrl_vision] Using Gazebo as input source." << endl;

            nh.param<string>("Gazebo_topic", Gazebo_topic_, "/gazebo/ground_truth/odom");

            // 【订阅】gazebo仿真真值
            Gazebo_sub_ = nh.subscribe<nav_msgs::Odometry>
                    (Gazebo_topic_, 100, Gazebo_cb);

            break;
        }

        case 3:{
            cout << "[px4ctrl_vision] Using T265 as input source." << endl;

            nh.param<string>("T265_topic", T265_topic_, "/t265/odom/sample");

            // 【订阅】t265估计位置
            T265_sub_ = nh.subscribe<nav_msgs::Odometry>
                    (T265_topic_, 100, T265_cb);

            break;
        }

        case 4: {
            cout << "[px4ctrl_vision] Using VICON as input source." << endl;

            nh.param<string>("subject_name", subject_name, "p450");
            nh.param<string>("segment_name", segment_name, "p450");

            // VICON
            VICON_sub_ = nh.subscribe<geometry_msgs::TransformStamped>
                    ("/vicon/" + subject_name + "/" + segment_name, 1000, VICON_cb);

            break;
        }

        case 5:{
            cout << "[px4ctrl_vision] Using LIO as input source." << endl;

            nh.param<string>("LIO_topic", LIO_topic_, "/Odometry");

            // subscribe to odometry from LIO
            LIO_sub_ = nh.subscribe<nav_msgs::Odometry>
                    (LIO_topic_, 100, LIO_cb);

            break;
        }

        default:
            cout << "[px4ctrl_vision] Invalid input source! Exiting..." << endl;
            return -1;
    }

    // 【发布】无人机位置和偏航角 坐标系 ENU系
    // 本话题要发送飞控(通过mavros_extras/src/plugins/vision_pose_pose_estimate.cpp发送),
    // 对应Mavlink消息为VISION_POSITION_ESTIMATE(#102),
    // 对应的飞控中的uORB消息为vehicle_vision_pose_position.msg 及 vehicle_vision_pose_attitude.msg
    vision_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/vision_pose/pose", 10);
    odom_out_pub_ = nh.advertise<nav_msgs::Odometry>
            ("/mavros/odometry/out", 10);

    vision_pose_.header.stamp = ros::Time::now();
    vision_pose_.header.frame_id = "world";

    odom_out_.header.stamp = ros::Time::now();
    odom_out_.header.frame_id = "world";

    // frequency to publish
    ros::Rate rate(50.0);

    cout << "[px4ctrl_vision] estimator initialized! " << endl;

    /****** Main Loop ******/
    while (ros::ok()){
        // 回调一次 更新传感器状态
        ros::spinOnce();

        // 将采集的机载设备的定位信息及偏航角信息发送至飞控
        vision_pose_.header.stamp = ros::Time::now();
        vision_pose_pub_.publish(vision_pose_);

        rate.sleep();
    }

    return 0;
}
