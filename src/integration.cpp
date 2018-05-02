/*

author : Yudai Sadakuni

division sqlidar pointcloud

*/

#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const PointCloud2ConstPtr& pc0, const PointCloud2ConstPtr& pc1, const PointCloud2ConstPtr& pc2)
{
    printf("ALL GREEN\n");

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "integration");
    ros::NodeHandle n;

    message_filters::Subscriber<PointCloud2> zed0_sub(n, "/sq_lidar/points/tf/zed0", 1);
    message_filters::Subscriber<PointCloud2> zed1_sub(n, "/sq_lidar/points/tf/zed1", 1);
    message_filters::Subscriber<PointCloud2> zed2_sub(n, "/sq_lidar/points/tf/zed2", 1);

    TimeSynchronizer<PointCloud2, PointCloud2, PointCloud2> sync(zed0_sub, zed1_sub, zed2_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    ros::spin();

    return 0;
}
