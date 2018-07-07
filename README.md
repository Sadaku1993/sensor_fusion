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

## How to Build
```
$cd $HOME
$cd catkin_ws/src
$git clone git@github.com:Sadaku1993/sensor_fusion.git
$cd ..
$catkin_make
```

```
$cd $HOME
$cd catkin_ws/src
$git clone git@github.com:Sadaku1993/sensor_fusion.git
$cd ..
$catkin_make
```

## How to Run

## Calibration SQ LiDAR and ZED
Watch [calibration](https://github.com/Sadaku1993/sensor_fusion/tree/master/calibration)

## Coloring LiDAR PointCloud Using ZED
Watch [coloring](https://github.com/Sadaku1993/sensor_fusion/tree/master/coloring)

## DepthImage Using LiDAR Points
Watch [depthimage](https://github.com/Sadaku1993/sensor_fusion/tree/master/depthimage)
