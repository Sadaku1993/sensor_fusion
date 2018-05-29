/*
save points using odometry

Subscribe
    

Publish


author : Yudai Sadakuni
*/

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

typedef pcl::PointXYZI PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

CloudAPtr save_pc_ (new CloudA);
CloudAPtr old_pc_ (new CloudA);

nav_msgs::Odometry odom_;
nav_msgs::Odometry init_odom_;

nav_msgs::Odometry sq_time;

ros::Publisher pub;

ros::Publisher pub_sq_time;

Eigen::Matrix4f transform_matrix;
Eigen::Matrix4f inverse_transform_matrix;

int count_ = 0;
int save_num = 100;

bool init_lcl_flag = false;

float z_threshold = 30.0;

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

void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    cout<<"------------"<<endl;
    CloudAPtr single_pc_(new CloudA);
    pcl::fromROSMsg(*msg, *single_pc_);

    cout<<"single_pc_ : " <<single_pc_->points.size()<<endl;

    CloudAPtr output_pc (new CloudA);
    pcl::transformPointCloud(*single_pc_, *output_pc, transform_matrix);

    CloudAPtr output_pc_after (new CloudA);

    for(size_t i=0;i<single_pc_->points.size();i++){
		double distance = sqrt(pow(single_pc_->points[i].x, 2)+
							   pow(single_pc_->points[i].y, 2)+
							   pow(single_pc_->points[i].z, 2));
		if(distance < 30){
        //if(single_pc_->points[i].z <= z_threshold){
            PointA temp;
            output_pc_after->points.push_back(output_pc->points[i]);
        }
    }

	CloudAPtr output_save_pc (new CloudA);
	Eigen::Matrix4f inverse_transform_matrix = transform_matrix.inverse();
	pcl::transformPointCloud(*output_pc_after, *output_save_pc, inverse_transform_matrix);
	

    if(count_ < save_num){
        *save_pc_ += *output_save_pc;
        old_pc_ = output_save_pc;
    }else{
        int old_pc_size = (int)old_pc_->points.size();
        save_pc_->points.erase(save_pc_->points.begin(), save_pc_->points.begin()+old_pc_size);
        *save_pc_ += *output_save_pc;
		old_pc_ = output_save_pc;
    }

    sensor_msgs::PointCloud2 pc_;
	pcl::toROSMsg(*save_pc_, pc_);
	pc_.header.stamp = ros::Time::now();
    pc_.header.frame_id = msg->header.frame_id;
    pub.publish(pc_);

	count_++;

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lsl");
    ros::NodeHandle n;

    ros::Rate rate(20);

    ros::Subscriber sub_pc = n.subscribe("/cloud/tf", 30, pc_callback);
    ros::Subscriber sub_lcl = n.subscribe("/odom", 30, lcl_callback);
    pub = n.advertise<sensor_msgs::PointCloud2>("/cloud/lcl", 30);

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

    cout<<"start"<<endl;

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
