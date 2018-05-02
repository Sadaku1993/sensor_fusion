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


void callback(const PointCloud2ConstPtr& pc0, const PointCloud2ConstPtr& pc1, const PointCloud2ConstPtr& pc2)
{
    printf("ALL GREEN\n");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl0(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl2(new pcl::PointCloud<pcl::PointXYZRGB>);

    fromROSMsg(*pc0, *pcl0);
    fromROSMsg(*pc1, *pcl1);
    fromROSMsg(*pc2, *pcl2);

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
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "integration");
    ros::NodeHandle n;

    message_filters::Subscriber<PointCloud2> zed0_sub(n, "/sq_lidar/points/tf/zed0", 1);
    message_filters::Subscriber<PointCloud2> zed1_sub(n, "/sq_lidar/points/tf/zed1", 1);
    message_filters::Subscriber<PointCloud2> zed2_sub(n, "/sq_lidar/points/tf/zed2", 1);

    pub = n.advertise<PointCloud2>("/sq_lidar/points/coloured", 1);

    TimeSynchronizer<PointCloud2, PointCloud2, PointCloud2> sync(zed0_sub, zed1_sub, zed2_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    ros::spin();

    return 0;
}
