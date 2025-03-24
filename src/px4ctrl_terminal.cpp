/*
MIT License

Copyright (c) 2025 Eason Hua

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
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
    ctrl_cmd_out_.mode              = easondrone_msgs::ControlCommand::Hold;
    ctrl_cmd_out_.setpoint_type     = easondrone_msgs::ControlCommand::SETPOINT_TYPE_POSITION;
    ctrl_cmd_out_.coordinate_frame  = easondrone_msgs::ControlCommand::FRAME_LOCAL_NED;
    ctrl_cmd_out_.poscmd.position.x = 0;
    ctrl_cmd_out_.poscmd.position.y = 0;
    ctrl_cmd_out_.poscmd.position.z = 0;
    ctrl_cmd_out_.poscmd.yaw        = 0;

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
