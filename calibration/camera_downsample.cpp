#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <iostream>

#include <sensor_fusion/down_sampling.h>
#include <sensor_fusion/pub_cloud.h>

using namespace std;

ros::Publisher pub_ds_cloud;

double DS_SIZE;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);

    CloudAPtr ds_cloud(new CloudA);
    down_sampling(cloud, ds_cloud, DS_SIZE);

    pub_cloud(ds_cloud, msg->header, pub_ds_cloud);
}


int main(int argc, char**argv)
{
    ros::init(argc, argv, "camera_downsample");
    ros::NodeHandle n;

    n.getParam("camera/ds_size", DS_SIZE);

    ros::Subscriber sub = n.subscribe("/cloud", 10, pcCallback);
    pub_ds_cloud = n.advertise<sensor_msgs::PointCloud2>("/output/ds_cloud", 10);

    ros::spin();

    return 0;
}
