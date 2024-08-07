/*
    px4ctrl_terminal.cpp
    Author: Eason Hua
    Update Time: 2024.08.07
    Introduction:  test function for sending ControlCommand.msg
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
    Command_to_pub.Mode                       = easondrone_msgs::ControlCommand::Hold;
    Command_to_pub.Reference_State.Move_mode  = easondrone_msgs::PositionReference::XYZ_POS;
    Command_to_pub.Reference_State.Move_frame = easondrone_msgs::PositionReference::ENU_FRAME;

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
