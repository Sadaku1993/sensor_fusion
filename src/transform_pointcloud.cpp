/*

author : Yudai Sadakuni

transform pointcloud data
source_frame -> target_frame

*/

#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>


using namespace std;

ros::Time t;

string target_frame = "/centerlaser";
string source_frame = "/centerlaser_";

sensor_msgs::PointCloud pc_;
void Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc_);
    t = msg->header.stamp;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sq_transform");
    ros::NodeHandle n;
    
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber sub = n.subscribe("/sq_lidar/points", 10, Callback);
    ros::Publisher  pub = n.advertise<sensor_msgs::PointCloud2>("/sq_lidar/points/tf", 10);

    ros::Rate rate(20);

    while(ros::ok())
    {
        sensor_msgs::PointCloud pc_trans;
        sensor_msgs::PointCloud2 pc2_trans;
        try{
            listener.waitForTransform(target_frame.c_str(), source_frame.c_str(), t, ros::Duration(1.0));
            listener.transformPointCloud(target_frame.c_str(), t, pc_, source_frame.c_str(), pc_trans);
            sensor_msgs::convertPointCloudToPointCloud2(pc_trans, pc2_trans);
        }catch (tf::TransformException& ex) {
            ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        }

        pub.publish(pc2_trans);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

