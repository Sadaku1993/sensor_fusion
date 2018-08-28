#!/bin/bash

hander()
{
    sleep 1
}

trap hander SIGINT

TIME=$(date +%Y-%m-%d-%H-%M-%S)
CLOUD="/cloud/tf"
ODOM="/odom"
IMU="/imu/data"

LCL="/cloud/lcl"

ZED0="/zed0/left/image_rect_color/compressed /zed0/right/image_rect_color/compressed /zed0/left/camera_info  "
ZED1="/zed1/left/image_rect_color/compressed /zed1/right/image_rect_color/compressed /zed1/left/camera_info  "
ZED2="/zed2/left/image_rect_color/compressed /zed2/right/image_rect_color/compressed /zed2/left/camera_info  "
DEPTH="/zed0/depth/depth_registered/compressedDepth /zed1/depth/depth_registered/compressedDepth /zed2/depth/depth_registered/compressedDepth"

echo $TIME &
echo $CLOUD &
echo $ODOM &
echo $IMU &

echo $LCL &

echo $ZED0 &
echo $ZED1 &
echo $ZED2 &
echo $DEPTH &

/opt/ros/kinetic/bin/rosbag record $ODOM $IMU -O /home/amsl/$TIME.bag
