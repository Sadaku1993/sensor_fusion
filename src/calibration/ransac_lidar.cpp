/* 
 * author : Yudai Sadakuni
 *
 * plane segmentation 
 *
 */

#include <ros/rros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointNormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

ros::Publisher pub_plane;

void plane_segmentation(CloudAPtr cloud, CloudA& plane);
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }

    plane.points.resize(inliers->indices.size());
    for(size_t i=0;i<inliers->indices.size();i++)
    {
        plane.points[i] = inliers->points[i];
    }
}

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    CloudAPtr intput (new CloudA);
    pcl::fromROSMsg(*msg, *input);

    CloudA plane;
    plane_segmentation(input, plane);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(plane, outout);
    output.header.stamp = ros::Time::now();
    output.header.frame_id = msg->header.frame_id;
    pub_plane.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ransac_lidar");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cloud", 10, pcCallback);
    pub = n.advertise<sensor_msgs::PointCloud2>("/plane", 10);

    ros::spin();

    return 0;
}
