#include <ros/ros.h>
#include "ros/package.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

// Initialization
ros::Publisher pub;
ros::Time t;

string LASER_TOPIC;
string CAMERA_INFO_TOPIC;
string IMAGE_TOPIC;
string TARGET_FRAME;
string SOUTCE_FRAME;
string PUBLISH_TOPIC;

sensor_msgs::PointCloud pc_tmp;

bool pc_flag = false;
bool camera_flag = false;
bool image_flag = false;

// Callback (SQ_LiDAR)
sensor_msgs::PointCloud pc_;
void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc_);
    t = msg->header.stamp;
    pc_flag = true;
}

//Callback (Camera Infomation)
sensor_msgs::CameraInfoConstPtr camera_;
void cameraCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    camera_ = msg;
    camera_flag = true;
}

// Callback(Image)
sensor_msgs::ImageConstPtr image_;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    image_ = msg;
    image_flag = true;
}
