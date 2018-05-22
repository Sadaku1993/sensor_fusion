#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transform.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <tf/tf.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#ifdef _OPENMP
#include <omp.h>
#endif


pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    sensor_msgs::PointCloud2ConstPtr pc;
    pc = msg;
    pub.publish(pc);
}

int main(int argc, char**argv)
{
    ros::init(argc, argc, "test_points_callback");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("cloud/tf", 1, pc_callback);
    pub = n.advertise<sensor_msgs::PointCloud2>("/cloud/test", 1);

    cout<<"start"<<endl;

    ros::spin();

    return 0;
}

