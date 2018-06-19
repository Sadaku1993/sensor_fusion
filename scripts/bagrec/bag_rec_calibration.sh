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
ZED0="/zed0/left/image_rect_color/compressed /zed0/left/camera_info /zed0/ds_cloud"
ZED1="/zed1/left/image_rect_color/compressed /zed1/left/camera_info /zed1/ds_cloud"
ZED2="/zed2/left/image_rect_color/compressed /zed2/left/camera_info /zed2/ds_cloud"
TF="/tf /tf_static"

echo $TIME &
echo $CLOUD &
echo $ODOM &
echo $IMU &
echo $ZED0 &
echo $ZED1 &
echo $ZED2 &
echo $TF &

/opt/ros/kinetic/bin/rosbag record $CLOUD $ODOM $IMU $ZED0 -O /home/amsl/$TIME.bag
