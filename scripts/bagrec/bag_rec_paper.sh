#!/bin/bash

hander()
{
    sleep 1
}

trap hander SIGINT

TIME=$(date +%Y-%m-%d-%H-%M-%S)
CLOUD="/cloud/tf"
LCL="/cloud/lcl"
ODOM="/odom"
IMU="/imu/data"
REALSENSE="/camera/color/camera_info /camera/color/image_raw"

echo $TIME &
echo $CLOUD &
echo $LCL &
echo $ODOM &
echo $IMU &
echo $REALSENSE &

/opt/ros/kinetic/bin/rosbag record $CLOUD $LCL $ODOM $IMU $REALSENSE -O /home/amsl/$TIME.bag
