#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc

# roscore
gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=50x12+0+0 &
sleep 1s

# Map Visualizer
# gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sensor_fusion map_publisher.launch" --geometry=50x12+0+300 &
# sleep 1s

# Transform 
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sensor_fusion transform_listener.launch" --geometry=50x12+0+600 &
sleep 1s

# Republish
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sensor_fusion republish.launch" --geometry=50x12+0+900 &
sleep 1s

# DepthImage Creater
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sensor_fusion depthimage_creater.launch" --geometry=50x12+600+0 &
sleep 1s

#Viewer
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sensor_fusion reference_viewer.launch" --geometry=50x12+600+300 &
sleep 1s

# TF
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sensor_fusion sq2_zed.launch" --geometry=50x12+1200+0 &
sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sensor_fusion zed0_dumy_tf.launch" --geometry=50x12+1200+300 &
sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sensor_fusion zed1_dumy_tf.launch" --geometry=50x12+1200+600 &
sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sensor_fusion zed2_dumy_tf.launch" --geometry=50x12+1200+900 &
sleep 60s

# bag
# gnome-terminal -e "/opt/ros/kinetic/bin/rosbag play /home/amsl/bagfiles/sq2/SII/depthimage/perfect_night.bag -r 1.0" --geometry=50x12+2000+500 &
# gnome-terminal -e "/opt/ros/kinetic/bin/rosbag play /home/amsl/bagfiles/sq2/depthimage/2018-07-17-01-08-13.bag -r 1.0" --geometry=50x12+2000+500 &
# gnome-terminal -e "/opt/ros/kinetic/bin/rosbag play /home/amsl/bagfiles/sq2/SII/depthimage/morning_v1.bag -r 1.0" --geometry=50x12+2000+500 &
gnome-terminal -e "/opt/ros/kinetic/bin/rosbag play /home/amsl/bagfiles/sq2/SII/depthimage/20180721_night_low.bag -r 1.0" --geometry=50x12+2000+500 &

# 朝 PCD 20180713
# gnome-terminal -e "/opt/ros/kinetic/bin/rosbag play /home/amsl/bagfiles/sq2/2018-07-13-06-53-13.bag -r 1.0" --geometry=50x12+2000+500 &
sleep 1s

# rviz
gnome-terminal -e '/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/.rviz/depth_image.rviz' --geometry=50x12+2000+750 &
sleep 1s
