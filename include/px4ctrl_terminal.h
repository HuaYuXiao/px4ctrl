//
// Created by hyx020222 on 8/6/24.
//

#ifndef PX4CTRL_PX4CTRL_TERMINAL_H
#define PX4CTRL_PX4CTRL_TERMINAL_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <set>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>

#include <easondrone_msgs/ControlCommand.h>
#include "cout_utils.h"


using namespace std;


const std::set<int> valid_modes = {0, 1, 2, 3, 4, 5, 6, 7};
//即将发布的command
easondrone_msgs::ControlCommand Command_to_pub;

//发布
ros::Publisher easondrone_ctrl_pub_;

void mainloop(){
    int Control_Mode = 0;
    bool valid_Control_Mode = false;
    int Move_mode = 0;
    bool valid_move_mode = false;
    int Move_frame = 0;
    bool valid_move_frame = false;
    float state_desired[4];
    bool valid_x_input = false;
    bool valid_y_input = false;
    bool valid_z_input = false;
    bool valid_yaw_input = false;

    while(ros::ok()){

        while (!valid_Control_Mode){
            cout << "--------------------------------" << endl;
            cout << "Enter command to mavros: " << endl;
            cout << "| 0 Arm  | 1 Offboard | 2 Takeoff | 3  Move  |" << endl;
            cout << "| 4 Hold | 5   Land   | 6 Manual  | 7 Disarm |" << endl;
            if (cin >> Control_Mode) {
                if (valid_modes.find(Control_Mode) != valid_modes.end()) {
                    valid_Control_Mode = true;
                }
                else{
                    string msg = "Invalid input! Please enter a valid command mode! \n";
                    cout_color(msg, RED_COLOR);
                }
            }
            else {
                // Clear error flags
                cin.clear();
                // Discard invalid input
                cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

                string msg = "Invalid input! Please enter an integer. \n";
                cout_color(msg, RED_COLOR);
            }
        }
        valid_Control_Mode = false;

        switch (Control_Mode){
            case easondrone_msgs::ControlCommand::Arm:{
                Command_to_pub.Mode = easondrone_msgs::ControlCommand::Arm;

                break;
            }

            case easondrone_msgs::ControlCommand::Offboard:{
                Command_to_pub.Mode = easondrone_msgs::ControlCommand::Offboard;

                break;
            }

            case easondrone_msgs::ControlCommand::Takeoff:{
                Command_to_pub.Mode = easondrone_msgs::ControlCommand::Takeoff;

                break;
            }

            case easondrone_msgs::ControlCommand::Move:{
                while (!valid_move_mode) {
                    cout << "Please choose Move_mode: 0 POS, 1 XY_POS_Z_VEL, 2 XY_VEL_Z_POS, 3 VEL" << endl;
                    if (cin >> Move_mode) {
                        if (Move_mode == 0 ||
                            Move_mode == 1 ||
                            Move_mode == 2 ||
                            Move_mode == 3) {
                            valid_move_mode = true;
                        }
                        else {
                            string msg = "Invalid input! Please enter a valid Move_mode. \n";
                            cout_color(msg, RED_COLOR);
                        }
                    }
                    else {
                        // Clear error flags
                        cin.clear();
                        // Discard invalid input
                        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

                        string msg = "Invalid input! Please enter an integer. \n";
                        cout_color(msg, RED_COLOR);
                    }
                }
                valid_move_mode = false;

                while (!valid_move_frame) {
                    cout << "Please choose Move_frame: 0 ENU, 1 Body" << endl;
                    if (cin >> Move_frame) {
                        if (Move_frame == 0 || Move_frame == 1) {
                            valid_move_frame = true;
                        }
                        else {
                            string msg = "Invalid input! Please enter 0 or 1. \n";
                            cout_color(msg, RED_COLOR);
                        }
                    }
                    else {
                        // Clear error flags
                        cin.clear();
                        // Discard invalid input
                        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

                        string msg = "Invalid input! Please enter an integer. \n";
                        cout_color(msg, RED_COLOR);
                    }
                }
                valid_move_frame = false;

                cout << "Please input the reference state [x y z yaw]: " << endl;

                while (!valid_x_input) {
                    cout << "setpoint_t[0] --- x [m] : " << endl;
                    if (cin >> state_desired[0]) {
                        valid_x_input = true;
                    }
                    else {
                        // Clear error flags
                        cin.clear();
                        // Discard invalid input
                        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

                        string msg = "Invalid input! Please enter a number. \n";
                        cout_color(msg, RED_COLOR);
                    }
                }
                valid_x_input = false;

                while (!valid_y_input) {
                    cout << "setpoint_t[1] --- y [m] : " << endl;
                    if (cin >> state_desired[1]) {
                        valid_y_input = true;
                    }
                    else {
                        // Clear error flags
                        cin.clear();
                        // Discard invalid input
                        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

                        string msg = "Invalid input! Please enter a number. \n";
                        cout_color(msg, RED_COLOR);
                    }
                }
                valid_y_input = false;

                while (!valid_z_input) {
                    cout << "setpoint_t[2] --- z [m] : " << endl;
                    if (cin >> state_desired[2]) {
                        if (state_desired[2] >= 0.0) {
                            valid_z_input = true;
                        }
                        else {
                            string msg = "Invalid input! Please enter a non-negative value. \n";
                            cout_color(msg, RED_COLOR);
                        }
                    }
                    else {
                        // Clear error flags
                        cin.clear();
                        // Discard invalid input
                        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

                        string msg = "Invalid input! Please enter a number. \n";
                        cout_color(msg, RED_COLOR);
                    }
                }
                valid_z_input = false;

                while (!valid_yaw_input) {
                    cout << "setpoint_t[3] --- yaw [deg] : " << endl;
                    if (cin >> state_desired[3]) {
                        // Check if yaw is within the range
                        if (state_desired[3] >= -180 && state_desired[3] < 180) {
                            valid_yaw_input = true;
                        }
                        else {
                            string msg = "Invalid input! Please enter a value between -180 and 180. \n";
                            cout_color(msg, RED_COLOR);
                        }
                    }
                    else {
                        // Clear error flags
                        cin.clear();
                        // Discard invalid input
                        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

                        string msg = "Invalid input! Please enter a number. \n";
                        cout_color(msg, RED_COLOR);
                    }
                }
                valid_yaw_input = false;

                Command_to_pub.Mode = easondrone_msgs::ControlCommand::Move;
                Command_to_pub.Reference_State.Move_mode = Move_mode;
                Command_to_pub.Reference_State.Move_frame = Move_frame;
                // yaw_rate control
                // Command_to_pub.Reference_State.Yaw_Rate_Mode = 1;
//                generate_com(Move_mode, state_desired);

                break;
            }

            case easondrone_msgs::ControlCommand::Hold:{
                Command_to_pub.Mode = easondrone_msgs::ControlCommand::Hold;

                break;
            }

            case easondrone_msgs::ControlCommand::Land:{
                Command_to_pub.Mode = easondrone_msgs::ControlCommand::Land;

                break;
            }

            case easondrone_msgs::ControlCommand::Manual:{
                Command_to_pub.Mode = easondrone_msgs::ControlCommand::Manual;

                break;
            }

            case easondrone_msgs::ControlCommand::Disarm:{
                Command_to_pub.Mode = easondrone_msgs::ControlCommand::Disarm;

                break;
            }

            default:{
                Command_to_pub.Mode = easondrone_msgs::ControlCommand::Hold;

                break;
            }
        }

        Command_to_pub.header.stamp = ros::Time::now();
        easondrone_ctrl_pub_.publish(Command_to_pub);

        cout_color("Command publish success!", GREEN_COLOR);
    }
}

#endif //PX4CTRL_PX4CTRL_TERMINAL_H
