#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc

# roscore
gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=50x12+0+0 &
sleep 1s

# DepthImage Creater
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sensor_fusion depthimage_creater.launch" --geometry=50x12+0+300 &
sleep 1s

# Transform 
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sensor_fusion transform_listener.launch" --geometry=50x12+0+600 &
sleep 1s

# Republish
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sensor_fusion republish.launch" --geometry=50x12+0+900 &
sleep 90s

# bag
gnome-terminal -e "/opt/ros/kinetic/bin/rosbag play /home/amsl/bagfiles/sq2/2018-07-07-23-50-02.bag -r 1" --geometry=50x12+2000+500 &
sleep 1s

# rviz
gnome-terminal -e '/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/.rviz/depth_image.rviz' --geometry=50x12+2000+750 &
sleep 1s
