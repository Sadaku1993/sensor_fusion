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

ros::Publisher pub_coloured;
ros::Publisher pub_velodyne;
ros::Publisher pub_camera;
ros::Publisher pub_image;
ros::Time t;
sensor_msgs::PointCloud pc_tmp;

string target_frame = "/zed/zed_left_camera";
// string target_frame = "/zed1/zed_center";
string source_frame = "/velodyne";

bool pc_flag = false;
bool camera_flag = false;
bool image_flag = false;

sensor_msgs::PointCloud2ConstPtr pc_;
void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pc_ = msg;
    pc_flag = true;
    pub_velodyne.publish(pc_);
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc_tmp);
    t = msg->header.stamp;

}

sensor_msgs::CameraInfoConstPtr camera_;
void cameraCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    camera_ = msg;
    camera_flag = true;
    pub_camera.publish(camera_);
}

sensor_msgs::ImageConstPtr image_;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    image_ = msg;
    image_flag = true;
    pub_image.publish(image_);
}

void coloring(sensor_msgs::PointCloud2 pcl_msg, const sensor_msgs::CameraInfoConstPtr& cinfo_msg, const sensor_msgs::ImageConstPtr& image_msg)
{
    ROS_INFO("\n\nColouring VELODYNE CLOUD!!");;
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
    //fromROSMsg(*pcl_msg, *pcl_cloud);
    fromROSMsg(pcl_msg, *trans_cloud);

    //tf::TransformListener listener;
    //tf::StampedTransform transform;
    
    //cout << "FRAME ID "<< pcl_cloud->header.frame_id << endl;
    // pcl_ros::transformPointCloud("stereo_camera", *pcl_cloud, *trans_cloud, listener);
 
   // pcl_ros::transformPointCloud (*pcl_cloud, *trans_cloud, transform);
    trans_cloud->header.frame_id = target_frame;

    pcl::copyPointCloud(*trans_cloud, *coloured);
    
    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pt = coloured->points.begin(); pt < coloured->points.end(); pt++)
    {
        //cv::Point3d pt_cv((*pt).x, (*pt).y, (*pt).z);
        if ((*pt).x<0) continue;
        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model_.project3dToPixel(pt_cv);

        if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows){
            // Copy colour to laser pointcloud
            (*pt).b = image.at<cv::Vec3b>(uv)[0];
            (*pt).g = image.at<cv::Vec3b>(uv)[1];
            (*pt).r = image.at<cv::Vec3b>(uv)[2];
        }

    }
    ROS_INFO("Publish coloured PC");

    // Publish coloured PointCloud
    sensor_msgs::PointCloud2 pcl_colour_ros;
    pcl::toROSMsg(*coloured, pcl_colour_ros);
    //pcl_colour_ros.header.stamp = pcl_msg->header.stamp ;
    pcl_colour_ros.header.stamp = t;
    pub_coloured.publish(pcl_colour_ros);


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sadaku_coloring");
    ros::NodeHandle n;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber pc_sub = n.subscribe("/velodyne_points",10,pcCallback);
    ros::Subscriber cinfo_sub = n.subscribe("/zed/left/camera_info",10,cameraCallback);
    ros::Subscriber image_sub = n.subscribe("/zed/left/image_rect_color",10,imageCallback);

    pub_coloured  = n.advertise<sensor_msgs::PointCloud2> ("velodyne_coloured",10);
    pub_velodyne = n.advertise<sensor_msgs::PointCloud2> ("velodyne/in",10);
    pub_camera   = n.advertise<sensor_msgs::CameraInfo> ("camera_info/in",10);
    pub_image    = n.advertise<sensor_msgs::Image> ("image/in",10);

    ros::Rate rate(20);
  

    while(ros::ok())
    {
        sensor_msgs::PointCloud pc_trans;
        sensor_msgs::PointCloud2 pc2_trans;
        try{
            listener.waitForTransform(target_frame.c_str(), source_frame.c_str(), t, ros::Duration(1.0));
            listener.transformPointCloud(target_frame.c_str(), t, pc_tmp, source_frame.c_str(), pc_trans);
            sensor_msgs::convertPointCloudToPointCloud2(pc_trans, pc2_trans);
            //listener.lookupTransform (target_frame.c_str(), source_frame.c_str(), t, transform);
        }catch (tf::TransformException& ex) {
            ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
            //return;
        }
        //if (pc_flag && camera_flag && image_flag) coloring(pc_, camera_, image_, transform);
        if (pc_flag && camera_flag && image_flag) coloring(pc2_trans, camera_, image_);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
