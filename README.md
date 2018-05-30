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
$roslaunch sq1_extra run_joy_for_bag.launch
```
and
```
$ssh tx2-0
$roslaunch zed-wrapper zed0.launch

$ssh tx2-1
$roslaunch zed-wrapper zed1.launch

$ssh tx2-2
$roslaunch zed-wrapper zed2.launch
```

### Launch Preprocessn Node
```
roslaunch sensor_fusion run.launch
```

### Launch DepthImage and ColoredCloud
```
$roslaunch sensor_fusion zed0.launch
$roslaunch sensor_fusion zed1.launch
$roslaunch sensor_fusion zed2.launch
```
