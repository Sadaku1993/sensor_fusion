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

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr save_pc_ (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr old_pc_ (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

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
	
	Eigen::Translation3f init_translation(reflect*odom_now.pose.pose.position.x, reflect*odom_now.pose.pose.position.y, reflect*odom_now.pose.pose.position.z);
	Eigen::AngleAxisf init_rotation_x(reflect*roll_now, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf init_rotation_y(reflect*pitch_now, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf init_rotation_z(reflect*yaw_now, Eigen::Vector3f::UnitZ());
	
	Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

	return init_guess;
}

void lcl_callback(nav_msgs::Odometry msg){
	if(init_lcl_flag){
		init_odom_ = msg;
		init_lcl_flag = false;
	}
	odom_ = msg;

	transform_matrix = create_matrix(odom_, 1.0);
}

void pc_callback(sensor_msgs::PointCloud2 msg){
	cout<<"-----------------------"<<endl;

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr single_pc_ (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::fromROSMsg(msg, *single_pc_);

	cout<<"single_pc_ : "<<single_pc_->points.size()<<endl;

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output_pc (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::transformPointCloud(*single_pc_, *output_pc, transform_matrix);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output_pc_after (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	size_t single_pc_size = single_pc_->points.size();
	for(size_t i=0; i<single_pc_size;i++){
		if(single_pc_->points[i].z <= z_threshold){
			pcl::PointXYZRGBNormal temp;
			temp.x = output_pc->points[i].x;
			temp.y = output_pc->points[i].y;
			temp.z = output_pc->points[i].z;
			temp.r = output_pc->points[i].r;
            temp.g = output_pc->points[i].g;
            temp.b = output_pc->points[i].b;
            //temp.intensity = output_pc->points[i].z;
			output_pc_after->points.push_back(temp);
		}

		cout<<"i"<<i<<endl;//iの個数知るため
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
	cout<<"save_pc_ : "<<save_pc_->points.size()<<endl;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output_save_pc (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	Eigen::Matrix4f inverse_transform_matrix = transform_matrix.inverse();
	pcl::transformPointCloud(*save_pc_, *output_save_pc, inverse_transform_matrix);

	sensor_msgs::PointCloud2 pc_;
	sensor_msgs::PointCloud2 vis_pc_;
	
    toROSMsg(*output_save_pc, pc_);
	pc_.header = msg.header;

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
	
	pub_pc = n.advertise<sensor_msgs::PointCloud2>("/sq_lidar/points/coloured/lcl", 1);
    pub_vis_pc = n.advertise<sensor_msgs::PointCloud2>("/sq_lidar/points/coloured/vis_lcl", 1);

	nav_msgs::Odometry init_odom;

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
	
    ros::spin();



	return 0;
}
