
#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/amsl/ros_catkin_ws/devel/setup.bash

source /home/amsl/.bashrc

#roscore
gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=50x12+0+0
sleep 1s

# sq1_extra
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sq1_extra sensor_fusion_sii.launch' --geometry=50x12+0+250 & 
# gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sq1_extra run_sq2_for_joy.launch' --geometry=50x12+0+250 & 
sleep 1s

# realsense
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch realsense2_camera rs_pointcloud.launch' --geometry=50x12+0+500 & 
sleep 1s

# Launch Sensor TF
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion laser_tf.launch' --geometry=50x12+0+500 &
sleep 1s

# laser to realsense
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion sq2_realsense.launch' --geometry=50x12+0+750 &
sleep 1s

# Transform PointCloud from /centerlaser_ to /centerlaser
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion laser_transform_pointcloud.launch' --geometry=50x12+250+0 &
sleep 1s

# threshold
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion threshold.launch' --geometry=50x12+250+250 &
sleep 1s

# lcl
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion lcl.launch' --geometry=50x12+250+500 &
sleep 1s

# transform publisher
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion odom_to_baselink.launch' --geometry=50x12+250+750 &
sleep 1s

# save flag
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion pub_saveflag.launch' --geometry=50x12+500+0 &
sleep 1s

# save node 
gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion node_saver.launch' --geometry=50x12+500+250 &
sleep 1s
# gnome-terminal -e '/opt/ros/kinetic/bin/roslaunch sensor_fusion save_cloud.launch' --geometry=50x12+250+500 &

###### Imu Complement
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch complement complement_pubtf.launch" --geometry=50x12+500+0 &
sleep 1s

####rviz########
gnome-terminal -e "/opt/ros/kinetic/bin/rosrun rviz rviz -d /home/amsl/.rviz/realsense.rviz" --geometry=50x12+1250+500 &

# bagrec
# gnome-terminal -e '/home/amsl/ros_catkin_ws/src/sensor_fusion/scripts/bagrec/bag_rec_paper.sh' --geometry=50x12+1200+500 &
