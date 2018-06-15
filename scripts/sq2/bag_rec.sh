#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc

# Launch SQ Lidar
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sq1_extra run_sq2_for_joy.launch' --geometry=50x12+0+0 &
sleep 1s

# Launch Zed 
gnome-terminal -e '/home/amsl/ros_catkin_ws/scripts/sensor_fusion/zed.sh' --geometry=50x12+0+250 &
sleep 1s

# Downsample Zed Cloud


# Launch Sensor TF
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion laser_tf.launch' --geometry=50x12+500+0 &
sleep 1s
gnome-terminal -e '/opt/ros/kinetic/bin/rosrun sensor_fusion laser_transform_pointcloud' --geometry=50x12+500+250 &
sleep 1s

# rviz
gnome-terminal -e '/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/.rviz/calibration.rviz' --geometry=50x12+500+500 &
sleep 5s

# bagrec
gnome-terminal -e '/home/amsl/ros_catkin_ws/scripts/sensor_fusion/bag_rec.sh' --geometry=50x12+500+500 &
