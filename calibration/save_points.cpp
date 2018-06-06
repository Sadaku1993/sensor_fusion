#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

using namespace std;

int count = 0;

void Callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);
	
	CloudA rm_ground;
	constructFullClouds(cloud, rm_ground);

	// Publish Coloured PointCloud
	sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(rm_ground, output);
    output.header.frame_id = msg->header.frame_id;
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "save_points");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cloud", 10, Callback);
    pub = n.advertise<sensor_msgs::PointCloud2>("/cloud/save", 10);
	
    ros::spin();

    return 0;
}
