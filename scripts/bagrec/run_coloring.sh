#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc

# Launch SQ Lidar
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sq1_extra run_sq2_for_joy.launch' --geometry=50x12+0+0 &
sleep 1s

# Launch Zed 
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/sensor_fusion/scripts/zed/zed0.sh' --geometry=50x12+0+250 &
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/sensor_fusion/scripts/zed/zed1.sh' --geometry=50x12+0+500 &
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/sensor_fusion/scripts/zed/zed2.sh' --geometry=50x12+0+750 &
sleep 1s

# Launch Sensor TF
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion laser_tf.launch' --geometry=50x12+250+0 &
sleep 1s
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion sq2_zed.launch' --geometry=50x12+250+250 &
sleep 1s

# Transform PointCloud from /centerlaser_ to /centerlaser
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion laser_transform_pointcloud.launch' --geometry=50x12+250+500 &
sleep 1s

# lcl
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion lcl.launch' --geometry=50x12+250+750 &
sleep 1s

# Divide PointCloud
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion division.launch' --geometry=50x12+500+0 &
sleep 1s

# Republish
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion republish.launch' --geometry=50x12+500+250 &
sleep 1s

# Coloring PointCloud
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion coloring_zed0.launch' --geometry=50x12+500+500 &
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion coloring_zed1.launch' --geometry=50x12+500+750 &
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion coloring_zed2.launch' --geometry=50x12+750+0 &
sleep 1s

# Integration
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion integration.launch' --geometry=50x12+750+250 &
sleep 1s

# rviz
gnome-terminal -e '/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/.rviz/coloring.rviz' --geometry=50x12+1200+750 &
sleep 5s

# bagrec
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/sensor_fusion/scripts/bagrec/bag_rec_coloring.sh' --geometry=50x12+1000+0 &
