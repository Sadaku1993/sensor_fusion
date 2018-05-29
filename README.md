# Sensor_Fusion
In this repository, I compiled the source code using ROS for the sensor fusion

## Overview
By integrating the data of LiDAR and camera, create teacher data sets for monocular camera.

## Requirements
- ROS Kinetic(ubuntu 16.04)
- Python2.7+
- [Opencv](https://opencv.org/)3.4+
- [PCL](https://pointcloud.org/)1.8+
- [zed-wrapper-ros](http://wiki.ros.org/zed-ros-wrapper)

## How to Run
### Calibration sq lidar and ZED
Under review

### Launch Sensor Node
```
$ roscd sensor_fusion/scripts
$ ./preprocessing.sh
```
or
```
roslaunch zed_wrapper zed.launch
roslaunch sq1_extra run_joy_for_bag.launch
roslaunch sensor_fusion republish.launch
roslaunch sensor_fusion sq2_zed.launch
roslaunch sensor_fusion laser_tf.launch
```

### Launch TF
```
roslaunch sensor_fusion run.launch
```

### Launch LiDAR and Camera Calibration Node
```
$roscd sensor_fusion/scripts
$./run.sh
```
or
```
$rosrun sensor_fusion laser_transform_pointcloud
$rosrun sensor_fusion save_points_odom
$rosrun sensor_fusion division
```

### Launch Depthimage
```
$roslaunch sensor_fusion depthimage_zed0.launch
$roslaunch sensor_fusion depthimage_zed1.launch
$roslaunch sensor_fusion depthimage_zed2.launch
```

### Launch Colouring PointCloud
```
$roslaunch sensor_fusion colouring_zed0.launch
$roslaunch sensor_fusion colouring_zed1.launch
$roslaunch sensor_fusion colouring_zed2.launch
$roslaunch sensor_fusion integration.launch
```
