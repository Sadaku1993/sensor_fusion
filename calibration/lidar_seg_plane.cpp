/* 
 * author : Yudai Sadakuni
 *
 * plane segmentation 
 *
 */


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <sensor_fusion/get_cluster_info.h>
#include <sensor_fusion/plane_segmentation.h>

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

ros::Publisher pub_plane;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    CloudAPtr input (new CloudA);
    pcl::fromROSMsg(*msg, *input);

    Clustering cluster_front;
    // CloudA cloud;
    // cluster.clustering(input, cloud);

    CloudA plane;
    plane_segmentation(input, plane);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(plane, output);
    output.header.stamp = ros::Time::now();
    output.header.frame_id = msg->header.frame_id;
    pub_plane.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ransac_lidar");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cloud", 10, pcCallback);
    pub_plane = n.advertise<sensor_msgs::PointCloud2>("/plane", 10);

    ros::spin();

    return 0;
}
