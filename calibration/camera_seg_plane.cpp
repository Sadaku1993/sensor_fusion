#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <iostream>

#include <sensor_fusion/down_sampling.h>
#include <sensor_fusion/plane_segmentation.h>
#include <sensor_fusion/clustering.h>
#include <sensor_fusion/outlier_removal.h>
#include <sensor_fusion/pub_cloud.h>

using namespace std;

ros::Publisher pub_pickup;
ros::Publisher pub_plane;
ros::Publisher pub_outlier;

// このパラメータで取得する点群のエリアを確定する
// キャリブレーションボードが収まる範囲に調整すること
double MIN_X, MAX_X;
double MIN_Y, MAX_Y;
double MIN_Z, MAX_Z;

double DISTANCE;


void pickup(CloudAPtr cloud, CloudAPtr& output)
{
    for(size_t i=0;i<cloud->points.size();i++){
        if( MIN_X < cloud->points[i].x && cloud->points[i].x < MAX_X
            && MIN_Y < cloud->points[i].y && cloud->points[i].y < MAX_Y
            && MIN_Z < cloud->points[i].z && cloud->points[i].z < MAX_Z)
            
            output->points.push_back(cloud->points[i]);
    }
}

void pcCallback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);

    CloudAPtr pickup_cloud(new CloudA);
    if(0<cloud->points.size()){
        pickup(cloud, pickup_cloud);
        pub_cloud(pickup_cloud, msg->header, pub_pickup);
    }

    CloudAPtr plane_cloud(new CloudA);
    if(0<pickup_cloud->points.size()){
        plane_segmentation(pickup_cloud, plane_cloud, DISTANCE);
        pub_cloud(plane_cloud, msg->header, pub_plane);
    }

    CloudAPtr outlier_cloud(new CloudA);
    if(0<plane_cloud->points.size()){
        outlier_removal(plane_cloud, outlier_cloud);
        pub_cloud(outlier_cloud, msg->header, pub_outlier);
    }
}


int main(int argc, char**argv)
{
    ros::init(argc, argv, "lidar_seg_plane");
    ros::NodeHandle n;
 
    n.getParam("min_x", MIN_X);
    n.getParam("max_x", MAX_X);
    n.getParam("min_y", MIN_Y);
    n.getParam("max_y", MAX_Y);
    n.getParam("min_z", MIN_Z);
    n.getParam("max_z", MAX_Z);

    n.getParam("distance"   , DISTANCE);

    ros::Subscriber sub = n.subscribe("/cloud", 10, pcCallback);
    pub_pickup = n.advertise<sensor_msgs::PointCloud2>("/output/pickup", 10);
    pub_plane  = n.advertise<sensor_msgs::PointCloud2>("/output/plane" , 10);
    pub_outlier = n.advertise<sensor_msgs::PointCloud2>("/output/outlier", 10);

    ros::spin();

    return 0;
}
