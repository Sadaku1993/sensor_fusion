#!/bin/bash

ssh -t -t tx2-4 << EOF
export ROS_MASTER_URI="http://192.168.0.142:11311"

sleep 1s

/opt/ros/kinetic/bin/roslaunch realsense2_camera realsense4.launch

sleep 1s
