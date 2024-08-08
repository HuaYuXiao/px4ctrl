/*
    px4ctrl_terminal.cpp
    Author: Eason Hua
    Update Time: 2024.08.07
    Introduction:  sending specific command to mavros via terminal
*/

#include "px4ctrl_terminal.h"

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　主函数　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv){
    ros::init(argc, argv, "px4ctrl_terminal");
    ros::NodeHandle nh;

    //　【发布】控制指令
    easondrone_ctrl_pub_ = nh.advertise<easondrone_msgs::ControlCommand>
            ("/easondrone/control_command", 10);

    // 初始化命令
    ctrl_cmd.mode = easondrone_msgs::ControlCommand::Hold;
    ctrl_cmd.frame = easondrone_msgs::ControlCommand::ENU;
    ctrl_cmd.poscmd.position.x = 0;
    ctrl_cmd.poscmd.position.y = 0;
    ctrl_cmd.poscmd.position.z = 0;
    ctrl_cmd.poscmd.yaw = 0;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    //cout.setf(ios::showpos);

    mainloop();

    return 0;
}
