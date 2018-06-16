/*

transform pointcloud data from laser_link to centerlaser

Subscribe:
    PointCloud2
    tf
Publish:
    PointCloud2

author : Yudai Sadakuni

*/

#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>


using namespace std;

class PointCloudTransform{
	private:
		ros::NodeHandle n;
		tf::TransformListener listener;
		tf::StampedTransform transform;
		ros::Publisher pub;
		ros::Subscriber sub;
		ros::Time t;
		sensor_msgs::PointCloud pc_;
		string target_frame = "/centerlaser";

	public:
		PointCloudTransform();
		void Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
};

PointCloudTransform::PointCloudTransform(){
    n.getParam("target_frame", target_frame);
	sub = n.subscribe("/cloud", 10, &PointCloudTransform::Callback, this);
	pub = n.advertise<sensor_msgs::PointCloud2>("/cloud/tf", 10);
}

void PointCloudTransform::Callback(const sensor_msgs::PointCloud2ConstPtr &msg){
	sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc_);
	t = msg->header.stamp;
	string source_frame = msg->header.frame_id;

	sensor_msgs::PointCloud pc_trans;
	sensor_msgs::PointCloud2 pc2_trans;

	try{
		listener.waitForTransform(target_frame.c_str(), source_frame.c_str(), t, ros::Duration(1.0));
		listener.transformPointCloud(target_frame.c_str(), t, pc_, source_frame.c_str(), pc_trans);
		sensor_msgs::convertPointCloudToPointCloud2(pc_trans, pc2_trans);
		pub.publish(pc2_trans);
	}catch (tf::TransformException& ex) {
		ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
	}
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_framsform_pointcloud");
	
	PointCloudTransform transform;
	ros::spin();

    return 0;
}

