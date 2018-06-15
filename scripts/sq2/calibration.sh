#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc

# Launch SQ Lidar
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sq1_extra run_sq2_for_joy.launch' --geometry=50x12+0+0 &
sleep 1s

# Launch Zed 
gnome-terminal -e '/home/amsl/ros_catkin_ws/scripts/sensor_fusion/zed0.sh' --geometry=50x12+0+250 &
sleep 1s
gnome-terminal -e '/home/amsl/ros_catkin_ws/scripts/sensor_fusion/zed1.sh' --geometry=50x12+0+500 &
sleep 1s
gnome-terminal -e '/home/amsl/ros_catkin_ws/scripts/sensor_fusion/zed2.sh' --geometry=50x12+0+750 &
sleep 1s

# Laser Pattern
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion calibration_lidar.launch' --geometry=50x12+0+600 &
sleep 1s

# rviz
gnome-terminal -e '/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/.rviz/calibration.rviz' --geometry=50x12+2000+750 &
sleep 1s
