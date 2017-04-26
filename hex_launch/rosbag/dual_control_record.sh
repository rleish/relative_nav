#!/bin/bash
rosbag record /imu/data /evart/heavy_ros/base /kinect_visual_odometry/vo_transformation /mikoCmd /truth_commands /mikoImu /alt_msgs /relative/global_pose /relative/states /relative/cur_edge/pose /k_scan /tf /hex_goal /hex_plan/navfn_planner/plan -b 4096
