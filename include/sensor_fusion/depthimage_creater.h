#ifndef _DEPTH_IMAGE_H_
#define _DEPTH_IMAGE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d_omp.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <sensor_fusion/Node.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

using namespace std;

typedef struct{
    double r, g, b;
} COLOR;

class DepthImage{
    private:
        ros::NodeHandle nh;
        image_transport::ImageTransport it;

        ros::Subscriber node_sub;
        image_transport::Publisher image_pub;

        //  Frame
        string global_frame;
        string laser_frame;
        string zed0_frame;
        string zed1_frame;
        string zed2_frame;
        
        // MAP File Path
        string FILE_PATH;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr map;

    public:
        DepthImage();

        void nodeCallback(const sensor_fusion::NodeConstPtr msg);

        void transform_pointcloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                                  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& trans_cloud,  
                                  tf::Transform transform,
                                  string target_frame,
                                  string source_frame);

        void depthimage_creater(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                                sensor_msgs::ImageConstPtr image_msg,
                                sensor_msgs::CameraInfoConstPtr cinfo_msg);


        void loadPCDFile(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                         string file_path);

        COLOR GetColor(double v, double vmin, double vmax);
};

DepthImage::DepthImage()
    : nh("~"), it(nh), map(new pcl::PointCloud<pcl::PointXYZRGBNormal>)

{
    nh.getParam("global_frame", global_frame);
    nh.getParam("laser_frame" , laser_frame);
    nh.getParam("zed0_frame"  , zed0_frame);
    nh.getParam("zed1_frame"  , zed1_frame);
    nh.getParam("zed2_frame"  , zed2_frame);

    nh.getParam("file_path", FILE_PATH);
    
    node_sub = nh.subscribe("/node", 10, &DepthImage::nodeCallback, this);

    image_pub = it.advertise("/depthimage", 10);

}


#include "depthimage_creater.cpp"

#endif
