# Sensor_Fusion
In this repository, I compiled the source code using ROS for the sensor fusion

## Overview
By integrating the data of LiDAR and camera, create teacher data sets for monocular camera.

## Requirements
- ROS Kinetic(ubuntu 16.04)
- Python2.7+
- [Opencv](https://opencv.org/)3.4
- [PCL](https://pointcloud.org/)1.8
- [zed-wrapper-ros](http://wiki.ros.org/zed-ros-wrapper)

## How to Run
### Calibration sq lidar and ZED
Under review

### Launch Sensor Node
```
$ roscd sensor_fusion/scripts
$ ./run.sh
```
or
```
roslaunch zed_wrapper zed.launch
roslaunch 
```
