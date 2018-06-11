#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc

# Launch preprocess node
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion calibration_lidar.launch' --geometry=50x12+0+600 &
sleep 1s

# rviz
gnome-terminal -e '/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/.rviz/calibration.rviz' --geometry=50x12+2000+750 &
sleep 1s
