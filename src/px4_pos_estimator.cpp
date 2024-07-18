#include "px4_vision.h"

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void send_to_fcu();
void pub_to_nodes(easondrone_msgs::DroneState State_from_fcu);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv){
    ros::init(argc, argv, "px4_pos_estimator");
    ros::NodeHandle nh("~");

    //　程序执行频率
    nh.param<float>("rate_hz", rate_hz_, 30);

    //读取参数表中的参数
    // 定位数据输入源 0 optitrack; 6 vicon; 1 激光SLAM; 2 gazebo; 3 T265; 9 outdoor
    nh.param<int>("input_source", input_source, 2);

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

    // VICON
    vicon_sub = nh.subscribe<geometry_msgs::TransformStamped>
            ("/vicon/" + subject_name + "/" + segment_name, 1000, vicon_cb);
    //  【订阅】t265估计位置
    t265_sub = nh.subscribe<nav_msgs::Odometry>
            ("/t265/odom/sample", 100, t265_cb);
    // 【订阅】gazebo仿真真值
    gazebo_sub = nh.subscribe<nav_msgs::Odometry>
            ("/mavros/local_position/odom", 100, gazebo_cb);
    // 【订阅】SLAM估计位姿
    slam_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/slam/pose", 100, slam_cb);
    // 【订阅】cartographer估计位置
    laser_sub = nh.subscribe<tf2_msgs::TFMessage>
            ("/tf", 100, laser_cb);
    // 【订阅】optitrack估计位置
    optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/vrpn_client_node/"+ object_name + "/pose", 100, optitrack_cb);
    // subscribe to odometry from VIO
    LIO_sub_ = nh.subscribe<nav_msgs::Odometry>
            ("/Odometry", 100, LIO_cb);

    // 【发布】无人机位置和偏航角 坐标系 ENU系
    //  本话题要发送飞控(通过mavros_extras/src/plugins/vision_pose_estimate.cpp发送),
    //  对应Mavlink消息为VISION_POSITION_ESTIMATE(#102),
    //  对应的飞控中的uORB消息为vehicle_vision_position.msg 及 vehicle_vision_attitude.msg
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/vision_pose/pose", 10);
    // 【发布】无人机状态量
    drone_state_pub = nh.advertise<easondrone_msgs::DroneState>
            ("/easondrone/drone_state", 10);
    // 【发布】无人机移动轨迹，用于RVIZ显示
    trajectory_pub = nh.advertise<nav_msgs::Path>
            ("/easondrone/drone_trajectory", 10);

    // 用于与mavros通讯的类，通过mavros接收来至飞控的消息【飞控->mavros->本程序】
    state_from_mavros _state_from_mavros;

    // 频率
    ros::Rate rate(rate_hz_);

    cout << "[control] estimator initialized" << endl;

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

void send_to_fcu(){
    geometry_msgs::PoseStamped vision;

    // TODO: convert to switch
    if (input_source == 0){
        // optitrack
        vision.pose.position.x = pos_drone_mocap[0];
        vision.pose.position.y = pos_drone_mocap[1];
        vision.pose.position.z = pos_drone_mocap[2];
        vision.pose.orientation.x = q_mocap.x();
        vision.pose.orientation.y = q_mocap.y();
        vision.pose.orientation.z = q_mocap.z();
        vision.pose.orientation.w = q_mocap.w();
      
        // 此处时间主要用于监测动捕，T265设备是否正常工作
        if(control_utils::get_time_in_sec(last_timestamp) > TIMEOUT_MAX){
            cout << "[estimator] Mocap Timeout" << endl;
        }
    }
    else if(input_source == 6){
        // VICON
        vision.pose.position.x = pos_drone_vicon[0];
        vision.pose.position.y = pos_drone_vicon[1];
        vision.pose.position.z = pos_drone_vicon[2];
        vision.pose.orientation.x = q_vicon.x();
        vision.pose.orientation.y = q_vicon.y();
        vision.pose.orientation.z = q_vicon.z();
        vision.pose.orientation.w = q_vicon.w();
    }
    else if (input_source == 3){
        vision.pose.position.x = pos_drone_t265[0];
        vision.pose.position.y = pos_drone_t265[1];
        vision.pose.position.z = pos_drone_t265[2];
        vision.pose.orientation.x = q_t265.x();
        vision.pose.orientation.y = q_t265.y();
        vision.pose.orientation.z = q_t265.z();
        vision.pose.orientation.w = q_t265.w();
    }
    else if (input_source == 1){
        // laser
        vision.pose.position.x = pos_drone_laser[0];
        vision.pose.position.y = pos_drone_laser[1];
        vision.pose.position.z = pos_drone_laser[2];
        //目前为二维雷达仿真情况，故z轴使用其他来源
        vision.pose.position.z = pos_drone_gazebo[2];
        vision.pose.orientation.x = q_laser.x();
        vision.pose.orientation.y = q_laser.y();
        vision.pose.orientation.z = q_laser.z();
        vision.pose.orientation.w = q_laser.w();
    }
    else if (input_source == 2){
        vision.pose.position.x = pos_drone_gazebo[0];
        vision.pose.position.y = pos_drone_gazebo[1];
        vision.pose.position.z = pos_drone_gazebo[2];
        vision.pose.orientation.x = q_gazebo.x();
        vision.pose.orientation.y = q_gazebo.y();
        vision.pose.orientation.z = q_gazebo.z();
        vision.pose.orientation.w = q_gazebo.w();
    }
    else if (input_source == 4){
        vision.pose.position.x = pos_drone_slam[0];
        vision.pose.position.y = pos_drone_slam[1];
        vision.pose.position.z = pos_drone_slam[2];
        vision.pose.orientation.x = q_slam.x();
        vision.pose.orientation.y = q_slam.y();
        vision.pose.orientation.z = q_slam.z();
        vision.pose.orientation.w = q_slam.w();
    }
    else if (input_source == 5){
        vision.pose.position.x = pos_LIO[0];
        vision.pose.position.y = pos_LIO[1];
        vision.pose.position.z = pos_LIO[2];
        vision.pose.orientation.x = q_LIO.x();
        vision.pose.orientation.y = q_LIO.y();
        vision.pose.orientation.z = q_LIO.z();
        vision.pose.orientation.w = q_LIO.w();
    }

    vision.header.stamp = ros::Time::now();
    vision_pub.publish(vision);
}

void pub_to_nodes(easondrone_msgs::DroneState State_from_fcu){
    // 发布无人机状态，具体内容参见 easondrone_msgs::DroneState
    Drone_State = State_from_fcu;
    Drone_State.header.stamp = ros::Time::now();
    // 户外情况，使用相对高度
    if(input_source == 9){
        Drone_State.position[2]  = Drone_State.rel_alt;
    }
    drone_state_pub.publish(Drone_State);

    // 发布无人机运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped drone_pos;
    drone_pos.header.stamp = ros::Time::now();
    drone_pos.header.frame_id = "map";
    drone_pos.pose.position.x = Drone_State.position[0];
    drone_pos.pose.position.y = Drone_State.position[1];
    drone_pos.pose.position.z = Drone_State.position[2];

    drone_pos.pose.orientation = Drone_State.attitude_q;

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
