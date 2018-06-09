// convert pcl to PointCloud2 and publish 

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void pub_cloud(CloudAPtr cloud, 
               std_msgs::Header header, 
               ros::Publisher pub)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.stamp = ros::Time::now();
    output.header.frame_id = header.frame_id;
    pub.publish(output);
}
