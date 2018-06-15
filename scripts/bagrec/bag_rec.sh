#!/bin/bash

hander()
{
    sleep 1
}

trap hander SIGINT

TIME=$(date +%Y-%m-%d-%H-%M-%S)
CLOUD="/cloud"
ODOM="/odom"
IMU="/imu/data"
ZED="/zed/left/image_rect_color/compressed /zed/left/camera_info /zed/ds_cloud"
TF="/tf /tf_static"

echo $TIME &
echo $CLOUD &
echo $ODOM &
echo $IMU &
echo $ZED &
echo $TF &

/opt/ros/kinetic/bin/rosbag record $CLOUD $ODOM $IMU $ZED -O /home/amsl/$TIME.bag
