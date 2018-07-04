#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc

# roscore
gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=50x12+0+0 &
sleep 1s

# Launch TF
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sensor_fusion base2laser.launch" --geometry=50x12+0+300
sleep 1s
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sensor_fusion zed0_dumy_tf.launch" --geometry=50x12+0+600
sleep 1s

# lcl
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sadayne_pointcloud lcl.launch" --geometry=50x12+600+0 &
sleep 1s

# map loader and NDT
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch kari_localization map_alignment.launch" --geometry=50x12+600+300 &
sleep 1s

# imu_complement
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch kari_complement imu_complement.launch" --geometry=50x12+600+600 &
sleep 1s

# ekf
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch ekf EKF.launch" --geometry=50x12+600+900 &
sleep 1s

# bag
gnome-terminal -e "/opt/ros/kinetic/bin/rosbag play /home/amsl/bagfiles/sq2/2018-07-05-00-16-38.bag" --geometry=50x12+2000+600 &
sleep 1s

# rviz
gnome-terminal -e '/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/.rviz/localization.rviz' --geometry=50x12+2000+900 &
sleep 1s
