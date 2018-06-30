#ifndef _SAVE_DATA_H_
#define _SAVE_DATA_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

typedef pcl::PointXYZI PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

#include <sensor_fusion/create_matrix.h>

using namespace std;
using namespace std_msgs;
using namespace nav_msgs;
using namespace sensor_msgs;
using namespace message_filters;
using namespace Eigen;



class SaveData{
    private:
        ros::NodeHandle nh;
        
        ros::Subscriber odom_sub;
        ros::Subscriber cloud_sub;

        ros::Subscriber zed0_optical_flow_sub;
        ros::Subscriber zed1_optical_flow_sub;
        ros::Subscriber zed2_optical_flow_sub;

        ros::Publisher emergency_pub;
        ros::Publisher optical_flow_reset_pub;

        // odometry
        bool odom_flag;
        Odometry odom;
        Odometry init_odom;
        Odometry old_odom;
        Odometry new_odom;
        Matrix4f transform_matrix;
        double distance;
        double threshold;

        // ZED0
        ImageConstPtr zed0_image;
        CameraInfoConstPtr zed0_cinfo;
        Matrix4f zed0_transform_matrix;
        tf::TransformListener zed0_listener;
        tf::StampedTransform  zed0_transform;
        bool zed0_data;
      
        // ZED1
        ImageConstPtr zed1_image;
        CameraInfoConstPtr zed1_cinfo;
        Matrix4f zed1_transform_matrix;
        tf::TransformListener zed1_listener;
        tf::StampedTransform  zed1_transform;
        bool zed1_data;

        // ZED2
        ImageConstPtr zed2_image;
        CameraInfoConstPtr zed2_cinfo;
        Matrix4f zed2_transform_matrix;
        tf::TransformListener zed2_listener;
        tf::StampedTransform  zed2_transform;
        bool zed2_data;

        // savecloud
        bool save_flag;
        int save_count;
        int count;
        CloudAPtr save_cloud;

        // Stop
        Bool emergency_flag;
        bool arrival;

    public:
        SaveData();

        void odomCallback(const OdometryConstPtr msg);
        void cloudCallback(const PointCloud2ConstPtr msg);

        typedef message_filters::sync_policies::ApproximateTime<Image, CameraInfo> ZED_sync_subs;
        
        //zed0 callback
        message_filters::Subscriber<Image> zed0_image_sub;
        message_filters::Subscriber<CameraInfo> zed0_cinfo_sub;
        message_filters::Synchronizer<ZED_sync_subs> zed0_sync;
        void zed0_callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info);
        //zed1 callback
        message_filters::Subscriber<Image> zed1_image_sub;
        message_filters::Subscriber<CameraInfo> zed1_cinfo_sub;
        message_filters::Synchronizer<ZED_sync_subs> zed1_sync;
        void zed1_callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info);
        //zed2 callback
        message_filters::Subscriber<Image> zed2_image_sub;
        message_filters::Subscriber<CameraInfo> zed2_cinfo_sub;
        message_filters::Synchronizer<ZED_sync_subs> zed2_sync;
        void zed2_callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info);

        // optical_flow callback
        void zed0_optical_flow_calback(const BoolConstPtr data);
        void zed1_optical_flow_calback(const BoolConstPtr data);
        void zed2_optical_flow_calback(const BoolConstPtr data);

        // save pointcloud at save point        
        void save_pointcloud(const PointCloud2::ConstPtr cloud);

        bool check_savepoint();

        void save_data();
};

SaveData::SaveData()
    : nh("~"),
      zed0_image_sub(nh, "/zed0/image", 10), zed0_cinfo_sub(nh, "/zed0/cinfo", 10),
      zed1_image_sub(nh, "/zed1/image", 10), zed1_cinfo_sub(nh, "/zed1/cinfo", 10),
      zed2_image_sub(nh, "/zed2/image", 10), zed2_cinfo_sub(nh, "/zed2/cinfo", 10),
      zed0_sync(ZED_sync_subs(10), zed0_image_sub, zed0_cinfo_sub),
      zed1_sync(ZED_sync_subs(10), zed1_image_sub, zed1_cinfo_sub),
      zed2_sync(ZED_sync_subs(10), zed2_image_sub, zed2_cinfo_sub)
{
    nh.getParam("threshold", threshold);

    odom_sub = nh.subscribe("/odom", 10, &SaveData::odomCallback, this);
    cloud_sub = nh.subscribe("/cloud", 10, &SaveData::cloudCallback, this);

    zed0_optical_flow_sub = nh.subscribe("/zed0/optical_flow", 10, &SaveData::zed0_optical_flow_calback, this);
    zed1_optical_flow_sub = nh.subscribe("/zed1/optical_flow", 10, &SaveData::zed1_optical_flow_calback, this);
    zed2_optical_flow_sub = nh.subscribe("/zed2/optical_flow", 10, &SaveData::zed2_optical_flow_calback, this);

	zed0_sync.registerCallback(boost::bind(&SaveData::zed0_callback, this, _1, _2));
	zed1_sync.registerCallback(boost::bind(&SaveData::zed1_callback, this, _1, _2));
	zed2_sync.registerCallback(boost::bind(&SaveData::zed2_callback, this, _1, _2));

    emergency_pub = nh.advertise<Bool>("/emergency_stop", 10);
    optical_flow_reset_pub = nh.advertise<Bool>("/optical_flow_reset", 10);

    // odom
    odom_flag = false;
    init_odom.header.frame_id = "/odom";
    init_odom.child_frame_id = "/base_link";
    init_odom.pose.pose.position.x = 0.0;
    init_odom.pose.pose.position.y = 0.0;
    init_odom.pose.pose.position.z = 0.0;
    init_odom.pose.pose.orientation.x = 0.0;
    init_odom.pose.pose.orientation.y = 0.0;
    init_odom.pose.pose.orientation.z = 0.0;
    init_odom.pose.pose.orientation.w = 0.0;
    odom = init_odom;
    distance = 0.0;

    //save cloud
    save_flag = false;
    save_count = 0;
    count = 0;

    // stop
    emergency_flag.data = false;
    arrival = false;

    // ZED
    zed0_data = true;
    zed1_data = true;
    zed2_data = true;
}


#include "save_data.cpp"

#endif
