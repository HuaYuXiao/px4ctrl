/***************************************************************************************************************************
* rc_control.cpp
*
* Author: X.S
*
* Update Time: 2021.6.12
*
* Introduction:  test function for rc swb
*
* bug:  Simulation at the same time start TERMINAL_CONTROL and RC_CONTROL, use TERMINAL_CONTROL unlock --> take-off, then use
* 
* RC_CONTROL control, control is normal, but then use TERMINAL_CONTROL to send instructions will not work

* Initial guess： RC_CONTROL control takes precedence over TERMINAL_CONTROL

* or PX4_sender subscribed to /Prometheus/control_command sent by the RC_Control and Terminal_Control nodes

***************************************************************************************************************************/

#include <ros/ros.h>
#include <iostream>
#include <mavros_msgs/RCIn.h>
#include <prometheus_msgs/ControlCommand.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>


#define NODE_NAME "rc_control"

using namespace std;

int channel = 0;

//即将发布的command
prometheus_msgs::ControlCommand Command_to_pub;

//发布
ros::Publisher swb_pub;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　函数声明　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

inline void timerCallback();

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  回调函数  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void swb_c(const mavros_msgs::RCIn::ConstPtr &msg)
{
    channel = msg->channels[5];
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　主函数　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rc_control");
    ros::NodeHandle nh;

    swb_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 100, swb_c);

    timerCallback();
    
	while(ros::ok())
    {
	    ros::spinOnce();
        
        if(channel > 1490 && channel < 1510)
        {
            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Hold;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            swb_pub.publish(Command_to_pub);
        }
        
        else if (channel > 1990 && channel < 2010)
        {
            Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            Command_to_pub.Reference_State.Move_mode  = prometheus_msgs::PositionReference::XYZ_VEL;
            Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
            Command_to_pub.Reference_State.time_from_start = -1;

            //Command_to_pub.Reference_State.position_ref[0] = 0;
            //Command_to_pub.Reference_State.position_ref[1] = 0;
            //Command_to_pub.Reference_State.position_ref[2] = 0;
            Command_to_pub.Reference_State.velocity_ref[0] = -0.1;
            Command_to_pub.Reference_State.velocity_ref[1] = 0;
            Command_to_pub.Reference_State.velocity_ref[2] = 0;
            swb_pub.publish(Command_to_pub);
        }
        else if (channel > 990 && channel < 1010)
        {
                        Command_to_pub.header.stamp = ros::Time::now();
            Command_to_pub.Mode = prometheus_msgs::ControlCommand::Move;
            Command_to_pub.Command_ID = Command_to_pub.Command_ID + 1;
            Command_to_pub.source = NODE_NAME;
            Command_to_pub.Reference_State.Move_mode  = prometheus_msgs::PositionReference::XYZ_VEL;
            Command_to_pub.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
            Command_to_pub.Reference_State.time_from_start = -1;

            //Command_to_pub.Reference_State.position_ref[0] = 0;
            //Command_to_pub.Reference_State.position_ref[1] = 0;
            //Command_to_pub.Reference_State.position_ref[1] = 0;
            Command_to_pub.Reference_State.velocity_ref[0] = 0.1;
            Command_to_pub.Reference_State.velocity_ref[1] = 0;
            Command_to_pub.Reference_State.velocity_ref[2] = 0;
            swb_pub.publish(Command_to_pub);
        }

        sleep(1.0);
        

    }
    return 0;
}

inline void timerCallback()
{
    cout << ">>>>>>>>>>>>>>>> Welcome to use Prometheus RC Control <<<<<<<<<<<<<<<<"<< endl;
    cout << endl;
    cout << ">>>Make sure to go into Offboard mode before using the SWB channel<<<<" <<endl;
    cout << endl;
    cout << ">>>>>>>>>>>>>>>>>>>>>>>  SWB上位----Vx = 0.1 m/s.  <<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << ">>>>>>>>>>>>>>>>>>>>>>>  SWB中位----Hold.  <<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << ">>>>>>>>>>>>>>>>>>>>>>>  SWB下位----Vx = 0.1 m/s.  <<<<<<<<<<<<<<<<<<<<" <<endl;
}
