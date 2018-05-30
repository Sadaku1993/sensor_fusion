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

## Hardware Spec
- PC
    - OS : Ubuntu16.04
    - Memory : 8GB
    - CPU : Intel® Core™ i7-7700
    - GPU : GeForce GTX 1050-Ti
- TX2
- Robot
    - Sensors
        - SQ-LiDAR(Meiji Univ)
        - ZED(Stereolabs)
        - AMU
    - Vehicle
        - Differetical drive

##How to Build
```
$cd $HOME
$cd catkin_ws/src
$git clone git@github.com:Sadaku1993/sensor_fusion.git
$cd ..
$catkin_make
```

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
$roslaunch sensor_fusion run.launch
```

### Launch DepthImage and ColoredCloud
```
$roslaunch sensor_fusion zed0.launch
$roslaunch sensor_fusion zed1.launch
$roslaunch sensor_fusion zed2.launch
```

## Run All Node (for amsl)
```
$roscd scripts/sensor_fusion
$./run.sh
```
