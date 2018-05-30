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
### Calibration SQ Lidar and ZED
Under review

### Launch Sensor Node
```
$roslaunch sq1_extra run_joy_for_bag.launch
```
```
$ssh 192.168.0.130
$export ROS_MASTER_URI=http://192.168.0.142:11311
$roslaunch zed-wrapper zed0.launch
```
```
$ssh 192.168.0.131
$export ROS_MASTER_URI=http://192.168.0.142:11311
$roslaunch zed-wrapper zed1.launch
```
```
$ssh 192.168.0.132
$export ROS_MASTER_URI=http://192.168.0.142:11311
$roslaunch zed-wrapper zed2.launch
```

### Launch Preprocess Node
```
roslaunch sensor_fusion run.launch
```

### Launch DepthImage and ColoredCloud
```
$roslaunch sensor_fusion zed0.launch
$roslaunch sensor_fusion zed1.launch
$roslaunch sensor_fusion zed2.launch
```


