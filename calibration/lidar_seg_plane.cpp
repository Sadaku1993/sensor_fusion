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

ros::Publisher pub_centroid;
ros::Publisher pub_points;
ros::Publisher pub_cloud;
ros::Publisher pub_plane;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    CloudAPtr input (new CloudA);
    pcl::fromROSMsg(*msg, *input);

    CloudAPtr cloud (new CloudA);
	CloudAPtr centroid (new CloudA);
	CloudAPtr points (new CloudA);
    
	// clustering and detect calibration board
	clustering(input,
			   cloud,
			   centroid,
			   points);
	
	// Plane Segmentation
    CloudA plane;
    if(0<cloud->points.size())
		plane_segmentation(cloud, plane);

	sensor_msgs::PointCloud2 centroid_;
	pcl::toROSMsg(*centroid, centroid_);
	centroid_.header.stamp = ros::Time::now();
	centroid_.header.frame_id = msg->header.frame_id;
	pub_centroid.publish(centroid_);

	sensor_msgs::PointCloud2 points_;
	pcl::toROSMsg(*points, points_);
	points_.header.stamp = ros::Time::now();
	points_.header.frame_id = msg->header.frame_id;
	pub_points.publish(points_);

	sensor_msgs::PointCloud2 cloud_;
	pcl::toROSMsg(*cloud, cloud_);
	cloud_.header.stamp = ros::Time::now();
	cloud_.header.frame_id = msg->header.frame_id;
	pub_cloud.publish(cloud_);

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

	pub_centroid = n.advertise<sensor_msgs::PointCloud2>("/centroid", 10);
	pub_points = n.advertise<sensor_msgs::PointCloud2>("/points", 10);
    pub_cloud = n.advertise<sensor_msgs::PointCloud2>("/calibration_board", 10);
    pub_plane = n.advertise<sensor_msgs::PointCloud2>("/plane", 10);
	
	ros::spin();

    return 0;
}
