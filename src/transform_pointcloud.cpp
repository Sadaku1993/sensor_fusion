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

string INPUT_TOPIC;
string OUTPUT_TOPIC;
string TARGET_FRAME;
string SOURCE_FRAME;

ros::Time t;

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

    n.getParam("/transform_pointcloud/target_frame", TARGET_FRAME);
    n.getParam("/transform_pointcloud/source_frame", SOURCE_FRAME);
    
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber sub = n.subscribe("cloud", 10, Callback);
    ros::Publisher  pub = n.advertise<sensor_msgs::PointCloud2>("cloud/tf", 10);
	
    ros::Rate rate(60);

    while(ros::ok())
    {
        sensor_msgs::PointCloud pc_trans;
        sensor_msgs::PointCloud2 pc2_trans;
        try{
            listener.waitForTransform(TARGET_FRAME.c_str(), SOURCE_FRAME.c_str(), t, ros::Duration(1.0));
            listener.transformPointCloud(TARGET_FRAME.c_str(), t, pc_, SOURCE_FRAME.c_str(), pc_trans);
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

