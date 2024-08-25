/*
    Created by hyx020222 on 2024.06.08
    Last modified on 2024.08.25
*/

#ifndef PX4CTRL_PX4CTRL_TERMINAL_H
#define PX4CTRL_PX4CTRL_TERMINAL_H

#include "px4ctrl_node.h"

using namespace Utils;

const std::set<int> valid_modes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
//即将发布的command
easondrone_msgs::ControlCommand ctrl_cmd_out_;

//发布
ros::Publisher easondrone_ctrl_pub_;

void mainloop(){
    int mode = 0;
    bool valid_mode = false;
    int frame = 0;
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
                ctrl_cmd_out_.frame = easondrone_msgs::ControlCommand::ENU;

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
                    cout << "Please choose frame: 0 ENU, 1 NED" << endl;
                    if (cin >> frame) {
                        if (frame == 0 || frame == 1) {
                            valid_frame = true;
                            ctrl_cmd_out_.frame = frame;
                        }
                        else {
                            string msg = "Invalid input! Require 0 or 1 ";
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
                            string msg = "Invalid input! Require value between (-180, 180) ";
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
