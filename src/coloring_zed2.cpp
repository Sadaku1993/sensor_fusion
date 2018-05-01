/*

author : Yudai Sadakuni

sensor calibration (sq_lidar and zed)


*/

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
#include <sensor_msgs/image_encodings.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

ros::Time t;

string target_frame = "/zed2/zed_left_camera";
string source_frame = "/centerlaser";

// subscribe data確認用flag
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


// Colouring Function
void colouring(sensor_msgs::PointCloud2 pc_msg, const sensor_msgs::CameraInfoConstPtr& cinfo_msg, const sensor_msgs::ImageConstPtr& image_msg)
{
    cout<<"ALL GREEN"<<endl;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sq_coloring_zed2");
    ros::NodeHandle n;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber pc_sub    = n.subscribe("/sq_lidar/points/left", 10, pcCallback); 
    ros::Subscriber cinfo_sub = n.subscribe("/zed2/zed/left/camera_info", 10, cameraCallback);
    ros::Subscriber image_sub = n.subscribe("/zed2/zed/left/image_rect_color", 10, imageCallback);

    ros::Rate rate(20);

    while(ros::ok())
    {
        sensor_msgs::PointCloud pc_trans;
        sensor_msgs::PointCloud2 pc2_trans;

        // Transform
        try{
            listener.waitForTransform(target_frame.c_str(), source_frame.c_str(), t, ros::Duration(1.0));
            listener.transformPointCloud(target_frame.c_str(), t, pc_, source_frame.c_str(), pc_trans);
            sensor_msgs::convertPointCloudToPointCloud2(pc_trans, pc2_trans);
        }catch (tf::TransformException& ex) {
            ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        }

        // coloring
        if(pc_flag && camera_flag && image_flag){
            colouring(pc2_trans, camera_, image_);
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
            
