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

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;


