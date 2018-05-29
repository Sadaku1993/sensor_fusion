#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#ifdef _OPENMP
#include <omp.h>
#endif

using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr save_pc_ (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr old_pc_ (new pcl::PointCloud<pcl::PointXYZI>);

nav_msgs::Odometry odom_;
nav_msgs::Odometry init_odom_;

ros::Publisher pub_pc;

Eigen::Matrix4f transform_matrix;
Eigen::Matrix4f inverse_transform_matrix;

int count_ = 0;
int save_num = 100;

bool init_lcl_flag = true;

float z_threshold = 30.0;

string POINTCLOUD_TOPIC;
string OUTPUT_TOPIC;
string ODOM_TOPIC;
string ODOM_PARENT_FRAME;
string ODOM_CHILD_FRAME;

Eigen::Matrix4f create_matrix(nav_msgs::Odometry odom_now){
	double roll_now, pitch_now, yaw_now;
	
	tf::Quaternion q_now(odom_now.pose.pose.orientation.x, odom_now.pose.pose.orientation.y, odom_now.pose.pose.orientation.z, odom_now.pose.pose.orientation.w);
	tf::Matrix3x3(q_now).getRPY(roll_now, pitch_now, yaw_now);
	// cout<<"roll_now : "<<roll_now<<", pitch_now : "<<pitch_now<<", yaw_now : "<<yaw_now<<endl;
	
	Eigen::Translation3f init_translation(odom_now.pose.pose.position.x, odom_now.pose.pose.position.y, odom_now.pose.pose.position.z);
	Eigen::AngleAxisf init_rotation_x(roll_now, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf init_rotation_y(pitch_now, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf init_rotation_z(yaw_now, Eigen::Vector3f::UnitZ());
	
	Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

	// Eigen::Matrix4f init_guess(Eigen::Matrix4f::Identity());
	return init_guess;
}

void lcl_callback(nav_msgs::Odometry msg){
	if(init_lcl_flag){
		init_odom_ = msg;
		init_lcl_flag = false;
		odom_ = msg;
		transform_matrix = create_matrix(odom_);
	}
	// odom_ = msg;
	// transform_matrix = create_matrix(odom_);
}

void pc_callback(sensor_msgs::PointCloud2 msg){
	cout<<"-----------------------"<<endl;

	pcl::PointCloud<pcl::PointXYZI>::Ptr single_pc_ (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(msg, *single_pc_);
	cout<<"single_pc_ : "<<single_pc_->points.size()<<endl;

	pcl::PointCloud<pcl::PointXYZI>::Ptr output_pc (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::transformPointCloud(*single_pc_, *output_pc, transform_matrix);

	pcl::PointCloud<pcl::PointXYZI>::Ptr output_pc_after (new pcl::PointCloud<pcl::PointXYZI>);
	size_t single_pc_size = single_pc_->points.size();
	for(size_t i=0; i<single_pc_size;i++){
		if(single_pc_->points[i].z <= z_threshold){
			pcl::PointXYZI temp;
			temp.x = output_pc->points[i].x;
			temp.y = output_pc->points[i].y;
			temp.z = output_pc->points[i].z;
			temp.intensity = output_pc->points[i].intensity;

			output_pc_after->points.push_back(temp);
		}
	}
		
	if(count_ < save_num){
		*save_pc_ += *output_pc_after;
		old_pc_ = output_pc_after;
	}
	else{
		int old_pc_size = (int)old_pc_->points.size();
		save_pc_->points.erase(save_pc_->points.begin(), save_pc_->points.begin()+old_pc_size);	
		*save_pc_ += *output_pc_after;
		old_pc_ = output_pc_after;
	}

	sensor_msgs::PointCloud2 pc_;
	toROSMsg(*output_pc_after, pc_);
	pc_.header.frame_id = msg.header.frame_id;
	pc_.header.stamp = ros::Time::now();
	pub_pc.publish(pc_);
	count_++;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "save_points_odom_2");
	ros::NodeHandle n;

	n.getParam("/odom_parent_frame", ODOM_PARENT_FRAME);
	n.getParam("/odom_child_frame", ODOM_CHILD_FRAME);

	ros::Subscriber sub_pc = n.subscribe("/cloud", 10, pc_callback);
	ros::Subscriber sub_lcl = n.subscribe("/odom", 10, lcl_callback);
	
	
	pub_pc = n.advertise<sensor_msgs::PointCloud2>("/output", 10);
	
	nav_msgs::Odometry init_odom;

	init_odom.header.frame_id = ODOM_PARENT_FRAME;
	init_odom.child_frame_id = ODOM_CHILD_FRAME;
	init_odom.pose.pose.position.x = 0.0;
	init_odom.pose.pose.position.y = 0.0;
	init_odom.pose.pose.position.z = 0.0;
	init_odom.pose.pose.orientation.x = 0.0;
	init_odom.pose.pose.orientation.y = 0.0;
	init_odom.pose.pose.orientation.z = 0.0;
	init_odom.pose.pose.orientation.w = 0.0;

	odom_ = init_odom;
	
	cout<<"start!!"<<endl;

	ros::spin();
	
	return 0;
}
