#!/bin/bash

ssh -t -t tx2-2 << EOF
export ROS_MASTER_URI="http://192.168.0.142:11311"

sleep 1s

/opt/ros/kinetic/bin/roslaunch sensor_fusion diff_image_zed2.launch

sleep 1s
