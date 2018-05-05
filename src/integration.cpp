/*

author : Yudai Sadakuni

division sqlidar pointcloud

*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>

using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pub;

bool flag0 = false;
bool flag1 = false;
bool flag2 = false;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl0(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl1(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl2(new pcl::PointCloud<pcl::PointXYZRGB>);

void zed0_callback(const PointCloud2ConstPtr& msg)
{
    fromROSMsg(*msg, *pcl0);
    flag0 = true;
}

void zed1_callback(const PointCloud2ConstPtr& msg)
{
    fromROSMsg(*msg, *pcl1);
    flag1 = true;
}

void zed2_callback(const PointCloud2ConstPtr& msg)
{
    fromROSMsg(*msg, *pcl2);
    flag2 = true;
}

void integrate()
{
    printf("ALL GREEN\n");

    pcl::PointCloud<pcl::PointXYZRGB> pcl_integrate;

    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator pt = pcl0->points.begin(); pt < pcl0->points.end(); pt++){
        pcl_integrate.points.push_back(*pt);
    }
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator pt = pcl1->points.begin(); pt < pcl1->points.end(); pt++){
        pcl_integrate.points.push_back(*pt);
    }
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator pt = pcl2->points.begin(); pt < pcl2->points.end(); pt++){
        pcl_integrate.points.push_back(*pt);
    }

    sensor_msgs::PointCloud2 pc2_integrate;
    pcl::toROSMsg(pcl_integrate, pc2_integrate);
    pc2_integrate.header.stamp = ros::Time::now();
    pc2_integrate.header.frame_id = "/centerlaser";
    pub.publish(pc2_integrate);

    // flag0 = false;
    // flag1 = false;
    // flag2 = false;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "integration");
    ros::NodeHandle n;

    ros::Subscriber sub0 = n.subscribe("/sq_lidar/points/tf/zed0", 10, zed0_callback);
    ros::Subscriber sub1 = n.subscribe("/sq_lidar/points/tf/zed1", 10, zed1_callback);
    ros::Subscriber sub2 = n.subscribe("/sq_lidar/points/tf/zed2", 10, zed2_callback);

    pub = n.advertise<PointCloud2>("/sq_lidar/points/coloured", 1);

    ros::Rate rate(30);
    while(ros::ok())
    {
        if(flag0 && flag1 && flag2) integrate();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
