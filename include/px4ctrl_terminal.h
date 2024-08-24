/*
    Created by hyx020222 on 2024.06.08
    Last modified on 2024.08.24
*/

#ifndef PX4CTRL_PX4CTRL_TERMINAL_H
#define PX4CTRL_PX4CTRL_TERMINAL_H

#include "px4ctrl_node.h"

using namespace Utils;

const std::set<int> valid_modes = {0, 1, 2, 3, 4, 5, 6, 7};
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
            cout << "--------------------------------" << endl;
            cout << "Enter command to mavros: " << endl;
            cout << "| 0 Arm  | 1 Offboard | 2 Takeoff | 3  Move  |" << endl;
            cout << "| 4 Hold | 5   Land   | 6 Manual  | 7 Disarm |" << endl;

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
            case easondrone_msgs::ControlCommand::Arm:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Arm;

                break;
            }

            case easondrone_msgs::ControlCommand::Offboard:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Offboard;

                break;
            }

            case easondrone_msgs::ControlCommand::Takeoff:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Takeoff;

                break;
            }

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

            case easondrone_msgs::ControlCommand::Hold:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Hold;

                break;
            }

            case easondrone_msgs::ControlCommand::Land:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Land;

                break;
            }

            case easondrone_msgs::ControlCommand::Manual:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Manual;

                break;
            }

            case easondrone_msgs::ControlCommand::Disarm:{
                ctrl_cmd_out_.mode = easondrone_msgs::ControlCommand::Disarm;
                ctrl_cmd_out_.frame = easondrone_msgs::ControlCommand::ENU;

                break;
            }
        }

        ctrl_cmd_out_.header.stamp = ros::Time::now();
        easondrone_ctrl_pub_.publish(ctrl_cmd_out_);

        cout_color("Command publish success!", GREEN_COLOR);
    }
}

#endif //PX4CTRL_PX4CTRL_TERMINAL_H
