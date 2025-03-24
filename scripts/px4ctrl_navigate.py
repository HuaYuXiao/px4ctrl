'''
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
'''

#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from quadrotor_msgs.msg import PositionCommand
from easondrone_msgs.msg import ControlCommand

odom_pos_ = np.zeros(3)

current_state = State()
# Create an instance of ControlCommand
ctrl_cmd = ControlCommand()


def odometryCallback(msg):
    global odom_pos_
    odom_pos_[0] = msg.pose.pose.position.x
    odom_pos_[1] = msg.pose.pose.position.y
    odom_pos_[2] = msg.pose.pose.position.z


def state_cb(msg):
    global current_state

    current_state = msg


def goal_cb(msg):
    print('------------------------')
    rospy.loginfo('Received cmd from /planning/direct_goal')

    if current_state.mode != 'OFFBOARD':
        rospy.logerr('Not Offboard, reject this request!')

        return

    rospy.loginfo('Request accepted!')

    global ctrl_cmd

    ctrl_cmd.header.stamp = rospy.Time.now()
    ctrl_cmd.poscmd.position.x = msg.pose.position.x
    ctrl_cmd.poscmd.position.y = msg.pose.position.y
    ctrl_cmd.poscmd.position.z = odom_pos_[2]
    ctrl_cmd.poscmd.yaw = 2 * np.arctan2(msg.pose.orientation.z, msg.pose.orientation.w)

    easondrone_ctrl_pub.publish(ctrl_cmd)


def planner_cb(msg):
    print('------------------------')
    rospy.loginfo('Received cmd from /planning/pos_cmd')

    if current_state.mode != 'OFFBOARD':
        rospy.logerr('Not Offboard, reject this request!')

        return

    rospy.loginfo('Request accepted!')

    global ctrl_cmd

    ctrl_cmd.header.stamp       = rospy.Time.now()
    ctrl_cmd.setpoint_type      = msg.setpoint_type
    ctrl_cmd.coordinate_frame   = msg.coordinate_frame
    ctrl_cmd.poscmd.position.x  = msg.position.x
    ctrl_cmd.poscmd.position.y  = msg.position.y
    ctrl_cmd.poscmd.position.z  = msg.position.z
    ctrl_cmd.poscmd.yaw         = msg.yaw

    easondrone_ctrl_pub.publish(ctrl_cmd)


if __name__ == '__main__':
    rospy.init_node('px4ctrl_navigate', anonymous=True)

    odom_sub    = (rospy.Subscriber
                ('/mavros/local_position/odom', Odometry, odometryCallback))
    state_sub   = (rospy.Subscriber
                ('/mavros/state', State, state_cb))
    goal_sub    = (rospy.Subscriber
                ('/planning/direct_goal', PoseStamped, goal_cb))
    planner_sub = (rospy.Subscriber
                ('/planning/pos_cmd', PositionCommand, planner_cb))

    easondrone_ctrl_pub = (rospy.Publisher
                        ('/easondrone/control_command', ControlCommand, queue_size=10))

    ctrl_cmd.mode               = ControlCommand.Move
    ctrl_cmd.setpoint_type      = ControlCommand.SETPOINT_TYPE_POSITION
    ctrl_cmd.coordinate_frame   = ControlCommand.FRAME_LOCAL_NED

    rospy.loginfo('px4ctrl_navigation Node Initialized!')

    rospy.spin()
