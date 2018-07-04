#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash
source /home/amsl/.bashrc

#roscore
gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=50x12+0+0
sleep 3s

# Launch Sensor TF
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion laser_tf.launch' --geometry=50x12+0+250 &
sleep 1s
# Transform PointCloud from /centerlaser_ to /centerlaser
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion laser_transform_pointcloud.launch' --geometry=50x12+0+500 &
sleep 1s
# LCL
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sensor_fusion lcl.launch" --geometry=50x12+0+750 
sleep 1s

# Launch Zed 
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion sq2_zed.launch' --geometry=50x12+250+0 &
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/sensor_fusion/scripts/zed/opticalflow_zed0.sh' --geometry=50x12+250+250 &
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/sensor_fusion/scripts/zed/opticalflow_zed1.sh' --geometry=50x12+250+500 &
gnome-terminal -e '/home/amsl/ros_catkin_ws/src/sensor_fusion/scripts/zed/opticalflow_zed2.sh' --geometry=50x12+250+750 &
sleep 1s

# republish
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion republish.launch ' --geometry=50x12+500+0 &
sleep 1s

gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion republish_optical_flow.launch ' --geometry=50x12+500+250 &
sleep 1s

# transform
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion transform_publisher.launch' --geometry=50x12+500+500 &
sleep 3s

###### localization ##########
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch kari_localization map_alignment.launch" --geometry=50x12+750+0 ##for si
sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch ekf EKF.launch" --geometry=50x12+750+250 ##for si
sleep 1s

###### obstacle map ##########
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sq_obstacle_map sq2_local_obs_map_kmb_filter.launch" --geometry=50x12+750+500 
sleep 1s

#####waypoint manager#######
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sq_waypoint_manager sq2_waypoint_manager.launch" --geometry=50x12+750+750
sleep 1s

###### path plan #######
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sq_local_planner sq2_local_path.launch" --geometry=50x12+900+0
sleep 1s

###### velocity publisher
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch sq1_extra sq2_sensor_fusion.launch" --geometry=50x12+900+250
sleep 1s

###### Imu Complement
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch kari_complement imu_complement.launch" --geometry=50x12+900+500 ##for si
sleep 1s

####rviz########
gnome-terminal -e "/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/.rviz/auto_savecloud.rviz" --geometry=40x12+900+750
sleep 3s

# SaveCloud
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion savecloud.launch' --geometry=50x12+1150+0 &
