/*
author : Yudai Sadakuni

publish depth image 

subscribe sq_lidar pointcloud
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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

ros::Time t;

image_transport::Publisher image_pub;

// Frame Name
string TARGET_FRAME;
string SOURCE_FRAME;
bool DEBUG;

// subscribe data確認用flag
bool pc_flag = false;
bool camera_flag = false;
bool image_flag = false;

typedef struct{
    double r, g, b;
} COLOUR;

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

// gradation depth data
COLOUR GetColour(double v, double vmin, double vmax)
{
    COLOUR c = {1.0, 1.0, 1.0}; // while
    double dv;

   if (v < vmin)
   v = vmin;
   if (v > vmax)
   v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
      c.r = 0;
      c.g = 4 * (v - vmin) / dv;
   } else if (v < (vmin + 0.5 * dv)) {
      c.r = 0;
      c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
   } else if (v < (vmin + 0.75 * dv)) {
      c.r = 4 * (v - vmin - 0.5 * dv) / dv;
      c.b = 0;
   } else {
      c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
      c.b = 0;
   }

   return(c);
}


// Colouring Function
void colouring(sensor_msgs::PointCloud2 pc_msg, const sensor_msgs::CameraInfoConstPtr& cinfo_msg, const sensor_msgs::ImageConstPtr& image_msg)
{
    printf("ALL GREEN\n");

    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
      cv_img_ptr = cv_bridge::toCvShare(image_msg);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    image = cv_bridge::toCvShare(image_msg)->image;

    image_geometry::PinholeCameraModel cam_model_;
    cam_model_.fromCameraInfo(cinfo_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>); // From ROS Msg
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>); // After transformation
    PointCloudXYZRGB::Ptr coloured = PointCloudXYZRGB::Ptr(new PointCloudXYZRGB); // For coloring purposes
    fromROSMsg(pc_msg, *trans_cloud);
    trans_cloud->header.frame_id = TARGET_FRAME;
    pcl::copyPointCloud(*trans_cloud, *coloured);
 
    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pt = coloured->points.begin(); pt < coloured->points.end(); pt++)
    {
        if ((*pt).x<0) continue;
        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model_.project3dToPixel(pt_cv);

        if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows){
            double range = sqrt( pow((*pt).x, 2.0) + pow((*pt).y, 2.0) + pow((*pt).z, 2.0));
            COLOUR c = GetColour(int(range/20*255.0), 0, 255);
            cv::circle(image, uv, 3, cv::Scalar(int(255*c.b),int(255*c.g),int(255*c.r)), -1);
		}
    }
	
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	image_pub.publish(msg);
	
	if(DEBUG){
    	cv::imshow("projection", image);
    	cv::waitKey(1);
	}

	pc_flag = false;
	camera_flag = false;
	image_flag = false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depthimage");
    ros::NodeHandle nh("~");
	
	nh.getParam("target_frame",TARGET_FRAME);
	nh.getParam("source_frame",SOURCE_FRAME);
	nh.getParam("debug", DEBUG);

    image_transport::ImageTransport it(nh);
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber pc_sub    = nh.subscribe("/cloud", 10, pcCallback); 
    ros::Subscriber cinfo_sub = nh.subscribe("/camera_info", 10, cameraCallback);
    ros::Subscriber image_sub = nh.subscribe("/image", 10, imageCallback);
	image_pub = it.advertise("/depthimage", 10);

    ros::Rate rate(10);

    while(ros::ok())
    {
        sensor_msgs::PointCloud pc_trans;
        sensor_msgs::PointCloud2 pc2_trans;

        // Transform
        try{
            listener.waitForTransform(TARGET_FRAME.c_str(), SOURCE_FRAME.c_str(), t, ros::Duration(1.0));
            listener.transformPointCloud(TARGET_FRAME.c_str(), t, pc_, SOURCE_FRAME.c_str(), pc_trans);
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
  
