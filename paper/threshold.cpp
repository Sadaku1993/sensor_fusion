#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;
double min_threshold;
double max_threshold;

void Callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr threshold_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::fromROSMsg(*msg, *input_cloud);

	for(size_t i=0;i<input_cloud->points.size();i++){
		double distance = sqrt(pow(input_cloud->points[i].x, 2)+ pow(input_cloud->points[i].y, 2) + pow(input_cloud->points[i].z, 2));
		if(min_threshold<distance && distance < max_threshold)
			threshold_cloud->points.push_back(input_cloud->points[i]);
	}

	sensor_msgs::PointCloud2 pc2;
	pcl::toROSMsg(*threshold_cloud, pc2);
	pc2.header.frame_id = msg->header.frame_id;
	pc2.header.stamp = ros::Time::now();
	pub.publish(pc2);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "threshold");
	ros::NodeHandle nh("~");

	nh.getParam("min_threshold", min_threshold);
	nh.getParam("max_threshold", max_threshold);

	ros::Subscriber sub = nh.subscribe("/cloud", 10, Callback);
	pub = nh.advertise<sensor_msgs::PointCloud2>("/output", 10);

	ros::spin();

	return 0;
}
