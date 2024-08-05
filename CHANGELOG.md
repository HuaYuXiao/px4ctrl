# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v3.6.2] - 2024-08-05
- [remove feature]: check for geofence
- [remove feature]: `easondrone_msgs::ControlCommand::Idle`

## [v3.6.1] - 2024-08-05
- [new feature]: support `VINS-Fusion`: `/vins_estimator/odometry` to `/mavros/vision_pose/pose`
- [remove support]: `cartographer`, `outdoor`
- [remove feature]: offset for position and orientation

## [v3.6.0] - 2024-07-26
- [new feature]: direct publish `/easondrone/control_command` from `/move_base_simple/goal`
- [remove support]: cmd from `/easondrone/control_command_station`

## [v3.5.2] - 2024-07-26
- [new feature]: support `FAST-LIO2`: `/Odometry` to `/mavros/vision_pose/pose`
- [remove feature]: euler angle estimate
- directly remap `nav_msgs::Odometry` to `/mavros/odometry/out`

## v3.5.1:
- OFFBOARD & arm with easondrone_msgs::ControlCommand::OFFBOARD_ARM
- [remove feature]: check for Command_ID
- [remove feature]: time_from_start
- launch for joy_node

## v3.5.0:
- replace `command_to_mavros` with `setpoint_raw_attitude_pub_`, `arming_client_`, `set_mode_client_`
- remove `state_from_mavros`

## v3.4.4:
- update `px4ctrl` to "cpp + h" format
- [remove feature]: `Drone_odom`

## v3.4.3: 
- remove: move along with trajectory
- remove: move publisher to rviz
- [remove feature]: agent_num in joy_node
- [remove support]: keyboard_control

## v3.4.2: 
- update odometry source of gazebo

## v3.4.1: 
- [remove feature]: `message_pub`

## v3.4.0: 
- support PID controller with `POS+VEL` loop

## v3.3.4: 
- update `rate_hz_` (removed in v3.5.3)

## v3.3.3:
- update `get_ref_pose_rviz`

## v3.3.2: 
- add support for `launch`

## v3.3.1: 
- [new feature]: support move mode `POS_VEL_ACC`

## v3.3.0: 
- [new feature]: support move mode `XYZ_VEL`

## v3.2.1: 
- [new feature]: catch invalid input of terminal control

## v2.0.0: 
- [new feature]: control type `NE`, `PID`, `UDE`

## v1.2.0: 
- import position offset

## v1.0.0: 
- [new feature]: `VICON`
