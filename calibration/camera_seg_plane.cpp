#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <iostream>

#include <sensor_fusion/down_sampling.h>
#include <sensor_fusion/pub_cloud.h>

using namespace std;

ros::Publisher pub_pickup;

string REALWORLD;
double THRESHOLD_X;
double THRESHOLD_Y;
double THRESHOLD_Z;
double DS_SIZE;

void pickup(CloudAPtr cloud, CloudAPtr& pickup_cloud)
{
    for(size_t i=0;i<cloud->points.size();i++){
        if(fabs(cloud->points[i].x) < THRESHOLD_X 
            && fabs(cloud->points[i].y) < THRESHOLD_Y
            && fabs(cloud->points[i].z) < THRESHOLD_Z )
        {
            pickup_cloud->points.push_back(cloud->points[i]);
        }
    }
}

void pcCallback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);

    CloudAPtr pickup_cloud(new CloudA);
    pickup(cloud, pickup_cloud);

    pub_cloud(pickup_cloud, msg->header, pub_pickup);
}


int main(int argc, char**argv)
{
    ros::init(argc, argv, "lidar_seg_plane");
    ros::NodeHandle n;

    n.getParam("threshold/x", THRESHOLD_X);
    n.getParam("threshold/y", THRESHOLD_Y);
    n.getParam("threshold/z", THRESHOLD_Z);

    ros::Subscriber sub = n.subscribe("/cloud", 10, pcCallback);
    pub_pickup   = n.advertise<sensor_msgs::PointCloud2>("/output/pickup", 10);

    ros::spin();

    return 0;
}
