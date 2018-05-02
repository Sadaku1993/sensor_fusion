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

#define PI 3.141592
#define UNIT_SIZE 1080

using namespace std;

// pcl::PointCloud<pcl::PointXYZ>::Ptr save_pc_ (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr save_pc_ (new pcl::PointCloud<pcl::PointXYZINormal>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr old_pc_ (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr old_pc_ (new pcl::PointCloud<pcl::PointXYZINormal>);

nav_msgs::Odometry odom_;
nav_msgs::Odometry init_odom_;

nav_msgs::Odometry sq_time;

ros::Publisher pub_pc;
ros::Publisher pub_vis_pc;

ros::Publisher pub_sq_time;

Eigen::Matrix4f transform_matrix;
Eigen::Matrix4f inverse_transform_matrix;

int count_ = 0;
int save_num =  100;

bool init_lcl_flag = true;

float z_threshold = 5.0;

Eigen::Matrix4f create_matrix(nav_msgs::Odometry odom_now, float reflect){
	double roll_now, pitch_now, yaw_now;
	
	tf::Quaternion q_now(odom_now.pose.pose.orientation.x, odom_now.pose.pose.orientation.y, odom_now.pose.pose.orientation.z, odom_now.pose.pose.orientation.w);
	tf::Matrix3x3(q_now).getRPY(roll_now, pitch_now, yaw_now);
	// cout<<"roll_now : "<<roll_now<<", pitch_now : "<<pitch_now<<", yaw_now : "<<yaw_now<<endl;
	
	Eigen::Translation3f init_translation(reflect*odom_now.pose.pose.position.x, reflect*odom_now.pose.pose.position.y, reflect*odom_now.pose.pose.position.z);
	Eigen::AngleAxisf init_rotation_x(reflect*roll_now, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf init_rotation_y(reflect*pitch_now, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf init_rotation_z(reflect*yaw_now, Eigen::Vector3f::UnitZ());
	
	Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

	// Eigen::Matrix4f init_guess(Eigen::Matrix4f::Identity());
	return init_guess;
}

void lcl_callback(nav_msgs::Odometry msg){
	if(init_lcl_flag){
		init_odom_ = msg;
		init_lcl_flag = false;
	}
	odom_ = msg;
	// odom_.pose.pose.position.x -= init_odom_.pose.pose.position.x;
	// odom_.pose.pose.position.y -= init_odom_.pose.pose.position.y;
	// odom_.pose.pose.position.z -= init_odom_.pose.pose.position.z;
	// odom_.pose.pose.orientation.x -= init_odom_.pose.pose.orientation.x;
	// odom_.pose.pose.orientation.y -= init_odom_.pose.pose.orientation.y;
	// odom_.pose.pose.orientation.z -= init_odom_.pose.pose.orientation.z;
	// odom_.pose.pose.orientation.w -= init_odom_.pose.pose.orientation.w;
	// cout<<"odom_.pose.pose.position.x : "<<odom_.pose.pose.position.x<<endl;
	// cout<<"odom_.pose.pose.position.y : "<<odom_.pose.pose.position.y<<endl;
	// cout<<"odom_.pose.pose.position.z : "<<odom_.pose.pose.position.z<<endl;
	// cout<<"odom_.pose.pose.orientation.x : "<<odom_.pose.pose.orientation.x<<endl;
	// cout<<"odom_.pose.pose.orientation.y : "<<odom_.pose.pose.orientation.y<<endl;
	// cout<<"odom_.pose.pose.orientation.x : "<<odom_.pose.pose.orientation.z<<endl;
	// cout<<"odom_.pose.pose.orientation.w : "<<odom_.pose.pose.orientation.w<<endl;

	transform_matrix = create_matrix(odom_, 1.0);
	// inverse_transform_matrix = create_matrix(odom_, -1.0);
	// cout<<"transform_matrix : "<<endl<<transform_matrix<<endl;


    //sq_time = odom_;
    //pub_vis_pc.publish(odom_);

}

void pc_callback(sensor_msgs::PointCloud2 msg){
	cout<<"-----------------------"<<endl;

	pcl::PointCloud<pcl::PointXYZINormal>::Ptr single_pc_ (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::fromROSMsg(msg, *single_pc_);
	//pcl::fromROSMsg(sensor_msgs::PointCloud2 cloud, );///内訳

	cout<<"single_pc_ : "<<single_pc_->points.size()<<endl;

	pcl::PointCloud<pcl::PointXYZINormal>::Ptr output_pc (new pcl::PointCloud<pcl::PointXYZINormal>);
	// pcl::transformPointCloud(*single_pc_, *output_pc, inverse_transform_matrix);
	pcl::transformPointCloud(*single_pc_, *output_pc, transform_matrix);

	pcl::PointCloud<pcl::PointXYZINormal>::Ptr output_pc_after (new pcl::PointCloud<pcl::PointXYZINormal>);
	size_t single_pc_size = single_pc_->points.size();
	for(size_t i=0; i<single_pc_size;i++){
		if(single_pc_->points[i].z <= z_threshold){
			pcl::PointXYZINormal temp;
			temp.x = output_pc->points[i].x;
			temp.y = output_pc->points[i].y;
			temp.z = output_pc->points[i].z;
			temp.intensity = output_pc->points[i].z;
			output_pc_after->points.push_back(temp);
		}

		cout<<"i"<<i<<endl;//iの個数知るため
	}
	if(count_ < save_num){
		*save_pc_ += *output_pc_after;
		// *save_pc_ += *output_pc;
		old_pc_ = output_pc_after;
	}
	else{
		int old_pc_size = (int)old_pc_->points.size();
		save_pc_->points.erase(save_pc_->points.begin(), save_pc_->points.begin()+old_pc_size);	
		*save_pc_ += *output_pc_after;
		// *save_pc_ += *output_pc;
		old_pc_ = output_pc_after;
	}
	cout<<"save_pc_ : "<<save_pc_->points.size()<<endl;
	// cout<<"save_pc_ : "<<save_pc_->width<<endl;
	// cout<<"save_pc_ : "<<save_pc_->height<<endl;
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr output_save_pc (new pcl::PointCloud<pcl::PointXYZINormal>);
	Eigen::Matrix4f inverse_transform_matrix = transform_matrix.inverse();
	pcl::transformPointCloud(*save_pc_, *output_save_pc, inverse_transform_matrix);

	// pcl::transformPointCloud(*save_pc_, *output_save_pc, transform_matrix);

	// pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	// approximate_voxel_filter.setLeafSize(0.3, 0.3, 0.3);
	// approximate_voxel_filter.setInputCloud(output_save_pc);
	// // approximate_voxel_filter.setInputCloud(save_pc_);
	// approximate_voxel_filter.filter(*filtered_pc);


	sensor_msgs::PointCloud2 pc_;
	sensor_msgs::PointCloud2 vis_pc_;
	// pcl::PCLPointCloud2 pcl_pointcloud2;
	// pcl::toPCLPointCloud2(*save_pc_, pcl_pointcloud2);
	// toROSMsg(*filtered_pc, pc_);
	toROSMsg(*output_save_pc, pc_);
	pc_.header = msg.header;
	// pc_.header.stamp = ros::Time::now();
	// pc_.header.frame_id = "/map";
	// toROSMsg(*single_pc_, pc_);

    vis_pc_ = pc_;

	pub_pc.publish(pc_);
	pub_vis_pc.publish(vis_pc_);
	count_++;


    
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "save_points");
	ros::NodeHandle n;

	ros::Subscriber sub_pc = n.subscribe("/sq_lidar/points/coloured", 1, pc_callback);
	ros::Subscriber sub_lcl = n.subscribe("/odom", 1, lcl_callback);
	// ros::Subscriber sub_lcl = n.subscribe("/odom/complement", 1, lcl_callback);
	
	
	pub_pc = n.advertise<sensor_msgs::PointCloud2>("/sq_lidar/points/coloured/lcl", 1);
    pub_vis_pc = n.advertise<sensor_msgs::PointCloud2>("/sq_lidar/points/coloured/vis_lcl", 1);
	

    //pub_sq_time = n.advertise<nav_msgs::Odometry>("/sq_lidar/time", 1);

	nav_msgs::Odometry init_odom;

	// init_odom.header.frame_id = "/odom";
	init_odom.header.frame_id = "/odom";
	init_odom.child_frame_id = "/base_link";
	init_odom.pose.pose.position.x = 0.0;
	init_odom.pose.pose.position.y = 0.0;
	init_odom.pose.pose.position.z = 0.0;
	init_odom.pose.pose.orientation.x = 0.0;
	init_odom.pose.pose.orientation.y = 0.0;
	init_odom.pose.pose.orientation.z = 0.0;
	init_odom.pose.pose.orientation.w = 0.0;

	odom_ = init_odom;
	
    
    cout<<"start!!"<<endl;
	// ros::Rate rate(40);
      // while (ros::ok()){
		// rate.sleep();
		// ros::spinOnce();
	// }
	
    ros::spin();



	return 0;
}
