#!/bin/bash

ssh -t -t tx2-0 << EOF
export ROS_MASTER_URI="http://192.168.0.142:11311"

sleep 1s

/opt/ros/kinetic/bin/roslaunch sensor_fusion downsample_zed0.launch

sleep 1s
