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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <sensor_fusion/get_cluster_info.h>
#include <sensor_fusion/plane_segmentation.h>
#include <sensor_fusion/outlier_removal.h>

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

ros::Publisher pub;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    CloudAPtr input (new CloudA);
    pcl::fromROSMsg(*msg, *input);


	// clustering and detect calibration board
    // CloudAPtr cloud (new CloudA);
	// CloudAPtr centroid (new CloudA);
	// CloudAPtr points (new CloudA);
	// clustering(input,
	// 		   cloud,
	// 		   centroid,
	// 		   points);
	
	// Plane Segmentation
    CloudAPtr plane(new CloudA);
    if(0<input->points.size())
		plane_segmentation(input, plane);

    // Outlier Removal
    CloudAPtr filtered(new CloudA);
    if(0<plane->points.size())
        outlier_removal(plane, filtered);

	sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*filtered, output);
    output.header.stamp = ros::Time::now();
    output.header.frame_id = msg->header.frame_id;
    pub.publish(output);
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
