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

#ifndef PX4CTRL_PX4CTRL_TERMINAL_H
#define PX4CTRL_PX4CTRL_TERMINAL_H

#include "px4ctrl_utils.h"

using namespace PX4CtrlFSM;
using namespace Utils;

const std::set<int> valid_modes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

void mainloop(){
    int mode = 0;
    bool valid_mode = false;
    int coordinate_frame = 0;
    bool valid_frame = false;
    bool valid_x_input = false;
    bool valid_y_input = false;
    bool valid_z_input = false;
    bool valid_yaw_input = false;

    while(ros::ok()){
        while (!valid_mode){
            cout << "----------------------------------------" << endl;
            cout << "Enter command to MAVROS: " << endl;
            std::cout << "| 0 \U0001F513 Arm     | 1 \U0001F512 Disarm    | 2 \U0001F6EB Takeoff| 3 \U0001F6EC Land     | 4\U0001F3E0 Return  |" << std::endl;
            std::cout <<"| 5 \U0001F579  Manual  | 6 \U0001F6AB Stabilized| 7 \U0001F6AB Acro   | 8 \U0001F6AB Rattitude| 9\U0001F6AB Altitude|" << std::endl;
            std::cout << "| 10\U0001F4BB Offboard| 11\U0001F6AB Position  | 12\U0001F4CC Hold   | 13\U0001F449 Move     |" << std::endl;

            if (cin >> mode) {
                if (valid_modes.find(mode) != valid_modes.end()) {
                    valid_mode = true;
                }
                else{
                    string msg = "Invalid input! Please enter a valid command mode! ";
                    cout_color(msg, RED_COLOR);
                }
            }
            else {
                string msg = "Invalid input! Please enter an integer. ";
                clear_cin(msg);
            }
        }
        valid_mode = false;

        switch (mode){
            // 0 Arm
            case easondrone_msgs::ControlCommand::Arm:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Arm;

                break;
            }

            // 1 Disarm
            case easondrone_msgs::ControlCommand::Disarm:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Disarm;

                break;
            }

            // 2 Takeoff
            case easondrone_msgs::ControlCommand::Takeoff:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Takeoff;

                break;
            }

            // 3 Land
            case easondrone_msgs::ControlCommand::Land:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Land;

                break;
            }

            // 4 Return
            case easondrone_msgs::ControlCommand::Return:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Return;

                break;
            }

            // 5 Manual
            case easondrone_msgs::ControlCommand::Manual:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Manual;

                break;
            }

            // 6 Stabilized
            case easondrone_msgs::ControlCommand::Stabilized:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Stabilized;

                break;
            }

            // 7 Acro
            case easondrone_msgs::ControlCommand::Acro:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Acro;

                break;
            }

            // 8 Rattitude
            case easondrone_msgs::ControlCommand::Rattitude:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Rattitude;

                break;
            }

            // 9 Altitude
            case easondrone_msgs::ControlCommand::Altitude:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Altitude;

                break;
            }

            // 10 Offboard
            case easondrone_msgs::ControlCommand::Offboard:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Offboard;

                break;
            }

            // 11 Position
            case easondrone_msgs::ControlCommand::Position:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Position;

                break;
            }

            // 12 Hold
            case easondrone_msgs::ControlCommand::Hold:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Hold;

                break;
            }

            // 13 Move
            case easondrone_msgs::ControlCommand::Move:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Move;
                
                while (!valid_frame) {
                    cout << "Please choose coordinate_frame:" << endl;
                    cout << "| 1 FRAME_LOCAL_NED | 7 FRAME_LOCAL_OFFSET_NED |" << endl;
                    cout << "| 8 FRAME_BODY_NED  | 9 FRAME_BODY_OFFSET_NED  |" << endl;
                    if (cin >> coordinate_frame) {
                        if (coordinate_frame == 1 || 
                            coordinate_frame == 7 ||
                            coordinate_frame == 8 ||
                            coordinate_frame == 9) {
                            valid_frame = true;
                            ctrl_cmd_out_.coordinate_frame = coordinate_frame;
                        }
                        else {
                            string msg = "Invalid input! Require 1 / 7 / 8 / 9";
                            cout_color(msg, RED_COLOR);
                        }
                    }
                    else {
                        string msg = "Invalid input! Require integer. ";
                        clear_cin(msg);
                    }
                }
                valid_frame = false;

                while (!valid_x_input) {
                    cout << "Enter position.x (unit: m) : " << endl;
                    if (cin >> ctrl_cmd_out_.poscmd.position.x) {
                        valid_x_input = true;
                    }
                    else {
                        string msg = "Invalid input! Require number ";
                        clear_cin(msg);
                    }
                }
                valid_x_input = false;

                while (!valid_y_input) {
                    cout << "Enter position.y (unit: m) : " << endl;
                    if (cin >> ctrl_cmd_out_.poscmd.position.y) {
                        valid_y_input = true;
                    }
                    else {
                        string msg = "Invalid input! Require number. ";
                        clear_cin(msg);
                    }
                }
                valid_y_input = false;

                while (!valid_z_input) {
                    cout << "Enter position.z (unit: m) : " << endl;
                    if (cin >> ctrl_cmd_out_.poscmd.position.z) {
                        if (ctrl_cmd_out_.poscmd.position.z >= 0.0) {
                            valid_z_input = true;
                        }
                        else {
                            cout_color("Invalid input! Require non-negative number!", RED_COLOR);
                        }
                    }
                    else {
                        string msg = "Invalid input! Require number";
                        clear_cin(msg);
                    }
                }
                valid_z_input = false;

                while (!valid_yaw_input) {
                    cout << "Enter yaw (unit: deg) : " << endl;
                    if (cin >> ctrl_cmd_out_.poscmd.yaw) {
                        // Check if yaw is within the range
                        if (abs(ctrl_cmd_out_.poscmd.yaw) <= 180) {
                            valid_yaw_input = true;
                            ctrl_cmd_out_.poscmd.yaw = ctrl_cmd_out_.poscmd.yaw / 180.0 * M_PI;
                        }
                        else {
                            string msg = "Invalid input! Require value between [-180.0, 180.0] ";
                            cout_color(msg, RED_COLOR);
                        }
                    }
                    else {
                        string msg = "Invalid input! Require number";
                        clear_cin(msg);
                    }
                }
                valid_yaw_input = false;

                break;
            }
        }

        ctrl_cmd_out_.header.stamp = ros::Time::now();
        easondrone_ctrl_pub_.publish(ctrl_cmd_out_);

        cout_color("Command publish success!", GREEN_COLOR);
    }
}

#endif //PX4CTRL_PX4CTRL_TERMINAL_H
