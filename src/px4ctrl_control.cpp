/*
    px4ctrl_control.cpp
    Author: Eason Hua
    last updated on 2024.08.24

    @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
    Stack and tested in Gazebo SITL
*/

#include "px4ctrl_node.h"

using namespace PX4CtrlFSM;
using namespace Utils;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv){
    ros::init(argc, argv, "px4ctrl_control");
    ros::NodeHandle nh("~");

    gp_origin_timer_ = nh.createTimer
            (ros::Duration(0.02), gpOriginCallback);

    state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>
            ("/mavros/local_position/odom", 10, odometryCallback);
    //【订阅】指令 本话题为任务模块生成的控制指令
    easondrone_ctrl_sub_ = nh.subscribe<easondrone_msgs::ControlCommand>
            ("/easondrone/control_command", 10, easondrone_ctrl_cb_);

    gp_origin_pub = nh.advertise<geographic_msgs::GeoPointStamped>
            ("/mavros/global_position/gp_origin", 10);
//    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
//            ("/mavros/setpoint_position/local", 10);
    // https://docs.ros.org/en/kinetic/api/mavros_msgs/html/msg/PositionTarget.html
    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10);
    // 【发布】经纬度以及高度位置 坐标系:WGS84坐标系
//    setpoint_raw_global_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
//            ("/mavros/setpoint_raw/global", 10);
    // 【发布】角度/角速度期望值 坐标系 ENU系
    //  本话题要发送至飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp发送), 对应Mavlink消息为SET_ATTITUDE_TARGET (#82), 对应的飞控中的uORB消息为vehicle_attitude_setpoint.msg（角度） 或vehicle_rates_setpoint.msg（角速度）
//    setpoint_raw_attitude_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>
//            ("/mavros/setpoint_raw/attitude", 10);

    // 【服务】解锁/上锁 本服务通过Mavros功能包 /plugins/command.cpp 实现
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    // 【服务】修改系统模式 本服务通过Mavros功能包 /plugins/command.cpp 实现
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    cout_color("Waiting for FCU connection...", YELLOW_COLOR);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    cout_color("FCU connected!", GREEN_COLOR);

    gp_origin.header.stamp = ros::Time::now();
    gp_origin.header.frame_id = "world";
    gp_origin.position.latitude = 0;
    gp_origin.position.longitude = 0;
    gp_origin.position.altitude = 0;

//    pose.pose.position.x = 0;
//    pose.pose.position.y = 0;
//    pose.pose.position.z = 0;

    /*
        uint16 type_mask
        uint16 IGNORE_PX = 1 # Position ignore flags
        uint16 IGNORE_PY = 2
        uint16 IGNORE_PZ = 4
        uint16 IGNORE_VX = 8 # Velocity vector ignore flags
        uint16 IGNORE_VY = 16
        uint16 IGNORE_VZ = 32
        uint16 IGNORE_AFX = 64 # Acceleration/Force vector ignore flags
        uint16 IGNORE_AFY = 128
        uint16 IGNORE_AFZ = 256
        uint16 FORCE = 512 # Force in af vector flag
        uint16 IGNORE_YAW = 1024
        uint16 IGNORE_YAW_RATE = 2048
     */
    pos_setpoint.type_mask = 0b100111111000; // 100 111 111 000  xyz + yaw
    /*
        uint8 coordinate_frame
        uint8 FRAME_LOCAL_NED = 1
        uint8 FRAME_LOCAL_OFFSET_NED = 7
        uint8 FRAME_BODY_NED = 8
        uint8 FRAME_BODY_OFFSET_NED = 9
    */
    pos_setpoint.coordinate_frame = 1;
    pos_setpoint.position.x = odom_pos_(0);
    pos_setpoint.position.y = odom_pos_(1);
    pos_setpoint.position.z = odom_pos_(2);
    pos_setpoint.yaw = odom_yaw_;

    // 初始化命令
    ctrl_cmd_in_.mode = easondrone_msgs::ControlCommand::Hold;
    ctrl_cmd_in_.frame = easondrone_msgs::ControlCommand::ENU;
    ctrl_cmd_in_.poscmd.position.x = odom_pos_(0);
    ctrl_cmd_in_.poscmd.position.y = odom_pos_(0);
    ctrl_cmd_in_.poscmd.position.z = odom_pos_(0);
    ctrl_cmd_in_.poscmd.yaw = odom_yaw_;

    cout_color("Send a few setpoints before starting...", YELLOW_COLOR);

    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
//        local_pos_pub.publish(pose);
        setpoint_raw_local_pub.publish(pos_setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    offb_set_mode.request.custom_mode = "AUTO.LOITER";

    arm_cmd.request.value = true;

    cout << "[px4ctrl_control] initialized!" << endl;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok()){
        cout << "------------------------" << endl;

        switch (ctrl_cmd_in_.mode){
            case easondrone_msgs::ControlCommand::Arm:{
                cout << "FSM_EXEC_STATE: Arm" << endl;

                if (!current_state.armed) {
                    arm_cmd.request.value = true;

                    pos_setpoint.coordinate_frame = 1;
                    pos_setpoint.position.x = odom_pos_(0);
                    pos_setpoint.position.y = odom_pos_(1);
                    pos_setpoint.position.z = odom_pos_(2);
                    pos_setpoint.yaw = odom_yaw_;

                    if (arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                        cout_color("Arm response success", YELLOW_COLOR);
                    }
                    else{
                        cout_color("Arm rejected by FCU", RED_COLOR);
                    }
                }
                else{
                    cout_color("Drone already Armed", GREEN_COLOR);
                }

                break;
            }

            case easondrone_msgs::ControlCommand::Offboard:{
                cout << "FSM_EXEC_STATE: Offboard" << endl;

                if (current_state.mode != "OFFBOARD") {
                    offb_set_mode.request.custom_mode = "OFFBOARD";

                    pos_setpoint.coordinate_frame = 1;
                    pos_setpoint.position.x = odom_pos_(0);
                    pos_setpoint.position.y = odom_pos_(1);
                    pos_setpoint.position.z = odom_pos_(2);
                    pos_setpoint.yaw = odom_yaw_;

                    if (set_mode_client.call(offb_set_mode) &&
                        offb_set_mode.response.mode_sent) {
                        cout_color("Offboard response sent", YELLOW_COLOR);
                    }
                    else{
                        cout_color("Offboard rejected by FCU", RED_COLOR);
                    }
                }
                else{
                    cout_color("Offboard already enabled", GREEN_COLOR);
                }

                break;
            }

            // 【Takeoff】 从摆放初始位置原地起飞至指定高度，偏航角也保持当前角度
            case easondrone_msgs::ControlCommand::Takeoff:{
                cout << "FSM_EXEC_STATE: Takeoff" << endl;

                // TODO: drone will fly over for about 1m
                if (odom_pos_(2) - 1.5 >= 0.2){
                    cout_color("Drone already Takeoff", GREEN_COLOR);

                    break;
                }

                if (current_state.mode != "AUTO.TAKEOFF") {
                    offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";

                    pos_setpoint.coordinate_frame = 1;
                    pos_setpoint.position.x = odom_pos_(0);
                    pos_setpoint.position.y = odom_pos_(1);
                    pos_setpoint.position.z = 1.5;
                    pos_setpoint.yaw = odom_yaw_;

                    if (set_mode_client.call(offb_set_mode) &&
                        offb_set_mode.response.mode_sent) {
                        cout_color("AUTO.TAKEOFF response sent", YELLOW_COLOR);
                    }
                    else{
                        cout_color("AUTO.TAKEOFF rejected by FCU!", RED_COLOR);
                    }
                }
                else{
                    cout_color("AUTO.TAKEOFF already enabled", GREEN_COLOR);
                }

                break;
            }

            // 【Move】 ENU系移动, 只能追踪位置
            case easondrone_msgs::ControlCommand::Move:{
                cout << "FSM_EXEC_STATE: Move" << endl;

                if (current_state.mode != "OFFBOARD") {
                    cout_color("Move command rejected, not in OFFBOARD mode", YELLOW_COLOR);

                    break;
                }
                else {
                    Eigen::Vector3d pos_offset;
                    pos_offset << ctrl_cmd_in_.poscmd.position.x - odom_pos_(0),
                                  ctrl_cmd_in_.poscmd.position.y - odom_pos_(1),
                                  ctrl_cmd_in_.poscmd.position.z - odom_pos_(2);
                    bool pos_ok = (pos_offset.norm() <= POS_ACCEPT);

                    // Normalize the difference to the range -pi to pi using boost
                    float yaw_offset = ctrl_cmd_in_.poscmd.yaw - odom_yaw_;
                    bool yaw_ok = false;
                    if (abs(yaw_offset) <= YAW_ACCEPT){
                        yaw_ok = true;
                    }
                    else if(abs(2 * M_PI - yaw_offset) <= YAW_ACCEPT){
                        yaw_ok = true;
                    };

                    if (pos_ok && yaw_ok){
                        cout_color("Already reach destination, skip move command!", GREEN_COLOR);

                        break;
                    }
                    else {
                        // TODO: other frames
                        pos_setpoint.coordinate_frame = 1;
                        pos_setpoint.position.x = ctrl_cmd_in_.poscmd.position.x;
                        pos_setpoint.position.y = ctrl_cmd_in_.poscmd.position.y;
                        pos_setpoint.position.z = ctrl_cmd_in_.poscmd.position.z;
                        pos_setpoint.yaw = ctrl_cmd_in_.poscmd.yaw;

                        // Use stringstream to concatenate the strings and float values
                        std::stringstream ss;
                        ss << "Moving to: "
                           << pos_setpoint.position.x << ", "
                           << pos_setpoint.position.y << ", "
                           << pos_setpoint.position.z << "; "
                           << pos_setpoint.yaw;

                        // Convert the stringstream to a string
                        std::string msg = ss.str();

                        // Print the result
                        cout_color(msg, YELLOW_COLOR);
                    }
                }

                break;
            }

            // 【Hold】 悬停。当前位置悬停
            case easondrone_msgs::ControlCommand::Hold:{
                cout << "FSM_EXEC_STATE: Hold" << endl;

                if (current_state.mode != "AUTO.LOITER") {
                    offb_set_mode.request.custom_mode = "AUTO.LOITER";

                    pos_setpoint.coordinate_frame = 1;
                    pos_setpoint.position.x = odom_pos_(0);
                    pos_setpoint.position.y = odom_pos_(1);
                    pos_setpoint.position.z = odom_pos_(2);
                    pos_setpoint.yaw = odom_yaw_;

                    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                        cout_color("AUTO.LOITER response sent", YELLOW_COLOR);
                    }
                    else{
                        cout_color("AUTO.LOITER rejected by FCU", RED_COLOR);
                    }
                }
                else{
                    cout_color("AUTO.LOITER already enabled", GREEN_COLOR);
                }

                break;
            }

            // 【Land】 降落。当前位置原地降落，降落后会自动上锁，且切换为mannual模式
            case easondrone_msgs::ControlCommand::Land:{
                cout << "FSM_EXEC_STATE: Land" << endl;

                if (odom_pos_(2) <= 0.2){
                    cout_color("Drone already Land", GREEN_COLOR);

                    break;
                }

                if (current_state.mode != "AUTO.LAND") {
                    offb_set_mode.request.custom_mode = "AUTO.LAND";

                    pos_setpoint.coordinate_frame = 1;
                    pos_setpoint.position.x = odom_pos_(0);
                    pos_setpoint.position.y = odom_pos_(1);
                    pos_setpoint.position.z = 0;
                    pos_setpoint.yaw = odom_yaw_;

                    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                        cout_color("AUTO.LAND response sent", YELLOW_COLOR);
                    }
                    else{
                        cout_color("AUTO.LAND rejected by FCU", RED_COLOR);
                    }
                }
                else{
                    cout_color("AUTO.LAND already enabled", GREEN_COLOR);
                }

                break;
            }

            // TODO: not tested yet, for real-life use only
            case easondrone_msgs::ControlCommand::Manual:{
                cout << "FSM_EXEC_STATE: Manual" << endl;

                if (current_state.mode != "MANUAL") {
                    offb_set_mode.request.custom_mode = "MANUAL";

                    pos_setpoint.coordinate_frame = 1;
                    pos_setpoint.position.x = odom_pos_(0);
                    pos_setpoint.position.y = odom_pos_(1);
                    pos_setpoint.position.z = odom_pos_(2);
                    pos_setpoint.yaw = odom_yaw_;

                    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                        cout_color("MANUAL response sent", YELLOW_COLOR);
                    }
                    else{
                        cout_color("MANUAL rejected by FCU", RED_COLOR);
                    }
                }
                else{
                    cout_color("MANUAL already enabled", GREEN_COLOR);
                }

                break;
            }

            // 【Disarm】 上锁
            case easondrone_msgs::ControlCommand::Disarm:{
                cout << "FSM_EXEC_STATE: Disarm" << endl;

                if (current_state.armed) {
                    arm_cmd.request.value = false;

                    pos_setpoint.coordinate_frame = 1;
                    pos_setpoint.position.x = odom_pos_(0);
                    pos_setpoint.position.y = odom_pos_(1);
                    pos_setpoint.position.z = 0;
                    pos_setpoint.yaw = odom_yaw_;

                    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                        cout_color("Disarm response success", YELLOW_COLOR);
                    }
                    else{
                        cout_color("Disarm rejected by FCU", RED_COLOR);
                    }
                }
                else{
                    cout_color("Drone already Disarmed", GREEN_COLOR);
                }

                break;
            }
        }

        pos_setpoint.header.stamp = ros::Time::now();
        setpoint_raw_local_pub.publish(pos_setpoint);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
