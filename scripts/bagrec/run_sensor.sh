#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc

# Launch SQ Lidar
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sq1_extra run_sq2_for_joy.launch' --geometry=50x12+0+0 &
sleep 1s

# Launch Zed 
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/sensor_fusion/scripts/zed/downsample_zed0.sh' --geometry=50x12+0+250 &
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/sensor_fusion/scripts/zed/downsample_zed1.sh' --geometry=50x12+0+500 &
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/sensor_fusion/scripts/zed/downsample_zed2.sh' --geometry=50x12+0+750 &
sleep 1s

# lcl
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion lcl.launch' --geometry=50x12+300+0 &
sleep 1s

# rviz
gnome-terminal -e '/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/.rviz/default.rviz' --geometry=50x12+1200+750 &
sleep 5s

# bagrec
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/sensor_fusion/scripts/bagrec/bag_rec_sensor.sh' --geometry=50x12+1000+0 &
