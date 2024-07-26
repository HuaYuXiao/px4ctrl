/***************************************************************************************************************************
 * px4_vision_pose.cpp
 *
 * Author: Qyp
* Maintainer: Eason Hua
* Update Time: 2024.07.19
 *
***************************************************************************************************************************/

#include "px4_vision_pose.h"

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void set_pose(geometry_msgs::Pose& pose, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
}

void send_to_fcu() {
    switch (input_source) {
        case 0:{
            // optitrack
            set_pose(vision_pose_.pose, pos_drone_mocap, q_mocap);

            vision_pose_.header.stamp = ros::Time::now();
            vision_pose_pub_.publish(vision_pose_);

            break;
        }

        case 1:{
            // laser
            set_pose(vision_pose_.pose, pos_drone_laser, q_laser);
            // TODO： 目前为二维雷达仿真情况，故z轴使用其他来源
//            vision_pose_.pose.position.z = pos_drone_gazebo[2];

            vision_pose_.header.stamp = ros::Time::now();
            vision_pose_pub_.publish(vision_pose_);

            break;
        }

        case 2:{
            // Gazebo
            odom_out_.header.stamp = ros::Time::now();
            odom_out_pub_.publish(odom_out_);

            break;
        }

        case 3:{
            // T265
            odom_out_.header.stamp = ros::Time::now();
            odom_out_pub_.publish(odom_out_);

            break;
        }

        case 4:{
            // slam
            vision_pose_.header.stamp = ros::Time::now();
            vision_pose_pub_.publish(vision_pose_);

            break;
        }

        case 5:{
            // LIO
            vision_pose_.header.stamp = ros::Time::now();
            vision_pose_pub_.publish(vision_pose_);

            break;
        }

        case 6:{
            // VICON
            vision_pose_.header.stamp = ros::Time::now();
            vision_pose_pub_.publish(vision_pose_);

            break;
        }

        default:{
            // Handle invalid input_source if necessary
            cout << "[estimator] Invalid input source" << endl;

            break;
        }
    }
}

// 发布无人机状态，具体内容参见 easondrone_msgs::DroneState
void pub_to_nodes(easondrone_msgs::DroneState State_from_fcu){
    State_from_fcu.header.stamp = ros::Time::now();
    // 户外情况，使用相对高度
    if(input_source == 9){
        State_from_fcu.position[2] = State_from_fcu.rel_alt;
    }
    drone_state_pub.publish(State_from_fcu);

    // 发布无人机运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped drone_pos;
    drone_pos.header.stamp = ros::Time::now();
    drone_pos.header.frame_id = "map";
    drone_pos.pose.position.x = State_from_fcu.position[0];
    drone_pos.pose.position.y = State_from_fcu.position[1];
    drone_pos.pose.position.z = State_from_fcu.position[2];

    drone_pos.pose.orientation = State_from_fcu.attitude_q;

    //发布无人机的位姿 和 轨迹 用作rviz中显示
    posehistory_vector_.insert(posehistory_vector_.begin(), drone_pos);
    if (posehistory_vector_.size() > TRA_WINDOW){
        posehistory_vector_.pop_back();
    }

    nav_msgs::Path drone_trajectory;
    drone_trajectory.header.stamp = ros::Time::now();
    drone_trajectory.header.frame_id = "map";
    drone_trajectory.poses = posehistory_vector_;
    trajectory_pub.publish(drone_trajectory);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv){
    ros::init(argc, argv, "px4_vision_pose");
    ros::NodeHandle nh("~");

    // 定位数据输入源
    nh.param<int>("input_source", input_source, 5);

    // TODO: redefined in pub_tp_fcu???
    nh.param<string>("child_frame_id",child_frame_id, "base_link");
    nh.param<string>("frame_id",frame_id, "map");

    // 动作捕捉设备中设定的刚体名字
    nh.param<string>("object_name", object_name, "p450");
    nh.param<string>("subject_name",subject_name, "p450");
    nh.param<string>("segment_name",segment_name, "p450");

    //　定位设备偏移量
    nh.param<float>("offset_x", pos_offset[0], 0.0);
    nh.param<float>("offset_y", pos_offset[1], 0.0);
    nh.param<float>("offset_z", pos_offset[2], 0.0);
    nh.param<float>("offset_yaw", yaw_offset, 0.0);
    nh.param<float>("offset_pitch", pitch_offset, 0.0);
    nh.param<float>("offset_roll", roll_offset, 0.0);

    nh.param<string>("LIO_topic", LIO_topic_, "/Odometry");
    nh.param<string>("T265_topic", T265_topic_, "/t265/odom/sample");

    // VICON
    VICON_sub_ = nh.subscribe<geometry_msgs::TransformStamped>
            ("/vicon/" + subject_name + "/" + segment_name, 1000, VICON_cb);
    //  【订阅】t265估计位置
    T265_sub_ = nh.subscribe<nav_msgs::Odometry>
            ("/t265/odom/sample", 100, T265_cb);
    // 【订阅】gazebo仿真真值
    Gazebo_sub_ = nh.subscribe<nav_msgs::Odometry>
            ("/mavros/local_position/odom", 100, Gazebo_cb);
    // 【订阅】SLAM估计位姿
    slam_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/slam/pose", 100, slam_cb);
    // 【订阅】cartographer估计位置
    laser_sub = nh.subscribe<tf2_msgs::TFMessage>
            ("/tf", 100, laser_cb);
    // 【订阅】optitrack估计位置
    optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/vrpn_client_node/"+ object_name + "/pose", 100, optitrack_cb);
    // subscribe to odometry from LIO
    LIO_sub_ = nh.subscribe<nav_msgs::Odometry>
            ("/Odometry", 100, LIO_cb);

    // 【发布】无人机位置和偏航角 坐标系 ENU系
    //  本话题要发送飞控(通过mavros_extras/src/plugins/vision_pose_pose_estimate.cpp发送),
    //  对应Mavlink消息为VISION_POSITION_ESTIMATE(#102),
    //  对应的飞控中的uORB消息为vehicle_vision_pose_position.msg 及 vehicle_vision_pose_attitude.msg
    vision_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/vision_pose/pose", 10);
    odom_out_pub_ = nh.advertise<nav_msgs::Odometry>
            ("/mavros/odometry/out", 10);
    // 【发布】无人机状态量
    drone_state_pub = nh.advertise<easondrone_msgs::DroneState>
            ("/easondrone/drone_state", 10);
    // 【发布】无人机移动轨迹，用于RVIZ显示
    trajectory_pub = nh.advertise<nav_msgs::Path>
            ("/easondrone/drone_trajectory", 10);

    // 用于与mavros通讯的类，通过mavros接收来至飞控的消息【飞控->mavros->本程序】
    state_from_mavros _state_from_mavros;

    // 频率
    ros::Rate rate(30);

    cout << "[px4ctrl] estimator initialized" << endl;

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while (ros::ok()){
        //回调一次 更新传感器状态
        ros::spinOnce();

        // 将采集的机载设备的定位信息及偏航角信息发送至飞控，根据参数input_source选择定位信息来源
        send_to_fcu();

        // 发布无人机状态至其他节点，如px4_pos_controller.cpp节点
        pub_to_nodes(_state_from_mavros._DroneState);

        rate.sleep();
    }
    return 0;
}
