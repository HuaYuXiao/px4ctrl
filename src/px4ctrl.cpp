/***************************************************************************************************************************
* px4_pos_controller.cpp
*
* Author: Qyp
* Maintainer: Eason Hua
// last updated on 2024.07.17
*
* Introduction:  PX4 Position Controller 
*         1. 从应用层节点订阅/easondrone/control_command话题（ControlCommand.msg），接收来自上层的控制指令。
*         2. 从px4_pos_estimator.cpp节点订阅无人机的状态信息（DroneState.msg）。
*         3. 调用位置环控制算法，计算加速度控制量，并转换为期望角度。（可选择cascade_PID, PID, UDE, passivity-UDE, NE+UDE位置控制算法）
*         4. 通过command_to_mavros.h将计算出来的控制指令发送至飞控（通过mavros包发送mavlink消息）
*         5. PX4固件通过mavlink_receiver.cpp接收该mavlink消息。
***************************************************************************************************************************/

#include "px4ctrl.h"

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv){
    ros::init(argc, argv, "px4ctrl");
    ros::NodeHandle nh("~");

    // 参数读取
    nh.param<float>("control/Takeoff_height", Takeoff_height_, 1.5);
    nh.param<float>("control/Disarm_height", Disarm_height_, 0.15);
    nh.param<float>("control/Land_speed", Land_speed_, 0.2);

    nh.param<float>("geo_fence/x_min", geo_fence_x[0], -8.0);
    nh.param<float>("geo_fence/x_max", geo_fence_x[1], 8.0);
    nh.param<float>("geo_fence/y_min", geo_fence_y[0], -5.0);
    nh.param<float>("geo_fence/y_max", geo_fence_y[1], 5.0);
    nh.param<float>("geo_fence/z_min", geo_fence_z[0], -0.3);
    nh.param<float>("geo_fence/z_max", geo_fence_z[1], 3.0);

    //【订阅】指令 本话题为任务模块生成的控制指令
    easondrone_ctrl_sub_ = nh.subscribe<easondrone_msgs::ControlCommand>
            ("/easondrone/control_command", 10, easondrone_ctrl_cb_);
    //【订阅】无人机状态 本话题来自px4_pos_estimator.cpp
    drone_state_sub = nh.subscribe<easondrone_msgs::DroneState>
            ("/easondrone/drone_state", 10, drone_state_cb);
    mavros_state_sub_ = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, mavros_state_cb);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>
            ("/mavros/local_position/odom", 10, odometryCallback);

    // 【发布】角度/角速度期望值 坐标系 ENU系
    //  本话题要发送至飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp发送), 对应Mavlink消息为SET_ATTITUDE_TARGET (#82), 对应的飞控中的uORB消息为vehicle_attitude_setpoint.msg（角度） 或vehicle_rates_setpoint.msg（角速度）
    setpoint_raw_attitude_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>
            ("/mavros/setpoint_raw/attitude", 10);
    //【发布】位置控制器的输出量:期望姿态
    att_ref_pub = nh.advertise<easondrone_msgs::AttitudeReference>
            ("/easondrone/control/attitude_reference", 10);

    // 【服务】解锁/上锁 本服务通过Mavros功能包 /plugins/command.cpp 实现
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    // 【服务】修改系统模式 本服务通过Mavros功能包 /plugins/command.cpp 实现
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    dt = 0.02;

    // 位置控制一般选取为50Hz，主要取决于位置状态的更新频率
    ros::Rate rate(100);

    // 用于与mavros通讯的类，通过mavros发送控制指令至飞控【本程序->mavros->飞控】
    command_to_mavros _command_to_mavros;

    // 位置控制器声明 可以设置自定义位置环控制算法
    pos_controller_cascade_PID pos_controller_cascade_pid;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>> px4ctrl Parameter <<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Takeoff_height   : "<< Takeoff_height_<<" [m] "<<endl;
    cout << "Disarm_height    : "<< Disarm_height_ <<" [m] "<<endl;
    cout << "Land_speed       : "<< Land_speed_ <<" [m/s] "<<endl;
    cout << "geo_fence_x : " << geo_fence_x[0] << " to " << geo_fence_x[1] << endl;
    cout << "geo_fence_y : " << geo_fence_y[0] << " to " << geo_fence_y[1] << endl;
    cout << "geo_fence_z : " << geo_fence_z[0] << " to " << geo_fence_z[1] << endl;

    // 初始化命令- 默认设置：Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_Now.Mode                                = easondrone_msgs::ControlCommand::Idle;
    Command_Now.Reference_State.Move_mode           = easondrone_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame          = easondrone_msgs::PositionReference::ENU_FRAME;

    // 记录启控时间
    ros::Time begin_time = ros::Time::now();
    float last_time = control_utils::get_time_in_sec(begin_time);

    cout << "[px4ctrl] controller initialized" << endl;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok()){
        // 当前时间
        cur_time = control_utils::get_time_in_sec(begin_time);
        dt = cur_time - last_time;
        dt = constrain_function2(dt, 0.008, 0.012);
        last_time = cur_time;

        //执行回调函数
        ros::spinOnce();

        // Check for geo fence: If drone is out of the geo fence, it will land now.
        if(!check_safety()){
            Command_Now.Mode = easondrone_msgs::ControlCommand::Land;
        }

        switch (Command_Now.Mode){
            // 【Idle】
            case easondrone_msgs::ControlCommand::Idle:{
                break;
            }

            // 怠速旋转，此时可以切入offboard模式，但不会起飞
            case easondrone_msgs::ControlCommand::OFFBOARD_ARM:{
                ROS_INFO("FSM_EXEC_STATE: OFFBOARD & ARM");

                if (mavros_state.mode != "OFFBOARD") {
                    offb_set_mode.request.custom_mode = "OFFBOARD";

                    if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                        ROS_INFO("Offboard enabled");
                    }
                }

                if (!mavros_state.armed) {
                    arm_cmd.request.value = true;

                    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed");
                    }
                }

                break;
            }

                // 【Takeoff】 从摆放初始位置原地起飞至指定高度，偏航角也保持当前角度
            case easondrone_msgs::ControlCommand::Takeoff:{
                //当无人机在空中时若受到起飞指令，则发出警告并悬停
                // if (_DroneState.landed == false){
                //     Command_Now.Mode = easondrone_msgs::ControlCommand::Hover;
                //     cout << "[control] The drone is in the air" << endl;
                // }

                if (Command_Last.Mode != easondrone_msgs::ControlCommand::Takeoff){
                    cout << "[control] Takeoff to desired point" << endl;
                    // 设定起飞位置
                    Takeoff_position[0] = odom_pos_[0];
                    Takeoff_position[1] = odom_pos_[1];
                    Takeoff_position[2] = odom_pos_[2];

                    //
                    Command_Now.Reference_State.Move_mode       = easondrone_msgs::PositionReference::XYZ_POS;
                    Command_Now.Reference_State.Move_frame      = easondrone_msgs::PositionReference::ENU_FRAME;
                    Command_Now.Reference_State.position_ref[0] = Takeoff_position[0];
                    Command_Now.Reference_State.position_ref[1] = Takeoff_position[1];
                    Command_Now.Reference_State.position_ref[2] = Takeoff_position[2] + Takeoff_height_;
                    Command_Now.Reference_State.yaw_ref         = odom_yaw_;
                }

                break;
            }

                // 【Hover】 悬停。当前位置悬停
            case easondrone_msgs::ControlCommand::Hover:{
                if (Command_Last.Mode != easondrone_msgs::ControlCommand::Hover){
                    Command_Now.Reference_State.Move_mode       = easondrone_msgs::PositionReference::XYZ_POS;
                    Command_Now.Reference_State.Move_frame      = easondrone_msgs::PositionReference::ENU_FRAME;
                    Command_Now.Reference_State.position_ref[0] = odom_pos_[0];
                    Command_Now.Reference_State.position_ref[1] = odom_pos_[1];
                    Command_Now.Reference_State.position_ref[2] = odom_pos_[2];
                    Command_Now.Reference_State.yaw_ref         = odom_yaw_; //rad
                }

                break;
            }

                // 【Land】 降落。当前位置原地降落，降落后会自动上锁，且切换为mannual模式
            case easondrone_msgs::ControlCommand::Land:{
                if (Command_Last.Mode != easondrone_msgs::ControlCommand::Land){
                    Command_Now.Reference_State.Move_mode       = easondrone_msgs::PositionReference::XY_POS_Z_VEL;
                    Command_Now.Reference_State.Move_frame      = easondrone_msgs::PositionReference::ENU_FRAME;
                    Command_Now.Reference_State.position_ref[0] = odom_pos_[0];
                    Command_Now.Reference_State.position_ref[1] = odom_pos_[1];
                    Command_Now.Reference_State.velocity_ref[2] = - Land_speed_; //Land_speed
                    Command_Now.Reference_State.yaw_ref         = odom_yaw_; //rad
                }

                //如果距离起飞高度小于10厘米，则直接切换为land模式；
                if(abs(odom_pos_[2] - Takeoff_position[2]) < Disarm_height_){
                    ROS_INFO("FSM_EXEC_STATE: LAND");

                    if (mavros_state.mode != "AUTO.LAND") {
                        offb_set_mode.request.custom_mode = "AUTO.LAND";

                        if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                            ROS_INFO("AUTO.LAND enabled");
                        }
                    }
                }

//                if(_DroneState.landed){
//                    Command_Now.Mode = easondrone_msgs::ControlCommand::Idle;
//                }

                break;
            }

                // 【Move】 ENU系移动。只有PID算法中才有追踪速度的选项，其他控制只能追踪位置
            case easondrone_msgs::ControlCommand::Move:{
                //对于机体系的指令,需要转换成ENU坐标系执行,且同一ID号内,只执行一次.
                if(Command_Now.Reference_State.Move_frame != easondrone_msgs::PositionReference::ENU_FRAME){
                    Body_to_ENU();
                }

                break;
            }

                // 【Disarm】 上锁
            case easondrone_msgs::ControlCommand::Disarm:{
                ROS_INFO("FSM_EXEC_STATE: DISARM");

                if (mavros_state.mode == "OFFBOARD") {
                    offb_set_mode.request.custom_mode = "MANUAL";

                    if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                        ROS_INFO("MANUAL enabled");
                    }
                }

                if (mavros_state.armed) {
                    arm_cmd.request.value = false;

                    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed");
                    }
                }

                break;
            }
        }

        //执行控制
        if(Command_Now.Mode != easondrone_msgs::ControlCommand::Idle){
            _ControlOutput = pos_controller_cascade_pid.pos_controller(_DroneState, Command_Now.Reference_State, dt);
        }

        throttle_sp[0] = _ControlOutput.Throttle[0];
        throttle_sp[1] = _ControlOutput.Throttle[1];
        throttle_sp[2] = _ControlOutput.Throttle[2];

        _AttitudeReference = control_utils::ThrottleToAttitude(throttle_sp, Command_Now.Reference_State.yaw_ref);

        //发送解算得到的期望姿态角至PX4
        _command_to_mavros.send_attitude_setpoint(_AttitudeReference);

        //发布期望姿态
        att_ref_pub.publish(_AttitudeReference);

        Command_Last = Command_Now;
        rate.sleep();
    }

    return 0;
}
