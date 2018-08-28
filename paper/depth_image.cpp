#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;

class Integrate{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;

        sensor_msgs::ImageConstPtr image_;
        sensor_msgs::CameraInfoConstPtr cinfo_;
        
        // odom to centerlaser
        tf::TransformListener global_listener;
        tf::StampedTransform  global_transform;

    public:
        Integrate();

        // odom callback
        void odomCallback(const nav_msgs::OdometryConstPtr msg);

        // camera callback
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> my_sync_subs;
        message_filters::Subscriber<sensor_msgs::Image>      image_sub;
        message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub;
        message_filters::Synchronizer<my_sync_subs> camera_sync;
        void camera_callback(const sensor_msgs::ImageConstPtr image, const sensor_msgs::CameraInfoConstPtr cinfo);

        // load PCD
        void load(CloudAPtr &cloud, string file_path);
};

Integrate::Integrate()
    : nh("~"),
      image_sub(nh, "/image", 10),
      cinfo_sub(nh, "/cinfo", 10),
      camera_sync(my_sync_subs(10), image_sub, cinfo_sub)
{
    nh.getParam("");
}

// global_frame(odom) to laser_frame
void Integrate::transform_listener()
{
	try{
		ros::Time now = ros::Time::now();
		global_listener.waitForTransform(global_frame, laser_frame, now, ros::Duration(1.0));
		global_listener.lookupTransform(global_frame, laser_frame,  now, global_transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
}

// transform pointcloud
void Integrate::transform_pointcloud(CloudAPtr cloud,
                                     CloudAPtr& trans_cloud,
                                     tf::Transform transform,
                                     string target_frame, 
                                     string source_frame)
{
    pcl_ros::transformPointCloud(*cloud, *trans_cloud, transform);
    trans_cloud->header.frame_id = target_frame;
    cout<<"---TF Cloud"<<" Frame:"<<trans_cloud->header.frame_id<<" Size:"<<trans_cloud->points.size()<<endl;
}



// camera callback
void Integrate::camera_callback(const sensor_msgs::ImageConstPtr image, const sensor_msgs::CameraInfoConstPtr cinfo)
{
    image_ = image;
    cinfo_ = cinfo;
}

// load PCDFile
void Integrate::load(CloudAPtr &cloud, string fime_path)
{
    cout<<"-----Load :" <<file_path<<endl;
    
    if (pcl::io::loadPCDFile<CloudA> (file_path, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("-----Couldn't read file\n");
    }
}
