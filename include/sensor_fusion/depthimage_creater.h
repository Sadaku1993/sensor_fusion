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

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

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

#include <omp.h>

#include <boost/thread/thread.hpp>
// #include <boost/shared_ptr.hpp>

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

typedef pcl::PointXYZRGBNormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;
using namespace Eigen;

typedef struct{
    double r, g, b;
} COLOR;

struct Cluster{
    float x; 
    float y; 
    float z;
    float width;
    float height;
    float depth;
    float curvature;
    Vector3f min_p;
    Vector3f max_p;
};

struct Clusters{
    Cluster data;
    CloudA centroid;
    CloudA points;
};

struct ROI{
    double x_offset;
    double y_offset;
    double width;
    double height;
};

class DepthImage{
    private:
        ros::NodeHandle nh;
        image_transport::ImageTransport it;

        ros::Subscriber node_sub;

        image_transport::Publisher zed0_pub;
        image_transport::Publisher zed1_pub;
        image_transport::Publisher zed2_pub;

        ros::Publisher zed0_raw_pub;
        ros::Publisher zed1_raw_pub;
        ros::Publisher zed2_raw_pub;

        ros::Publisher zed0_rmground_pub;
        ros::Publisher zed1_rmground_pub;
        ros::Publisher zed2_rmground_pub;

        ros::Publisher zed0_cluster_pub;
        ros::Publisher zed1_cluster_pub;
        ros::Publisher zed2_cluster_pub;

        ros::Publisher zed0_cloud_pub;
        ros::Publisher zed1_cloud_pub;
        ros::Publisher zed2_cloud_pub;

        //  Frame
        string global_frame;
        string laser_frame;
        string zed0_frame;
        string zed1_frame;
        string zed2_frame;
        
        // MAP File Path
        string OBSTACLE_PATH;
        string GROUND_PATH;
        CloudAPtr obstacle_map;
        CloudAPtr ground_map;

        // local cloud area
        int threshold;

        // min max
        double cell_size;
        int grid_dimentions;
        double height_threshold;

    public:
        DepthImage();

        void nodeCallback(const sensor_fusion::NodeConstPtr msg);

        void transform_pointcloud(CloudAPtr cloud,
                                  CloudAPtr& trans_cloud,  
                                  tf::Transform transform,
                                  string target_frame,
                                  string source_frame);

        void depthimage_creater(CloudAPtr obstacle_cloud,
                                CloudAPtr ground_cloud,
                                sensor_msgs::ImageConstPtr image_msg,
                                sensor_msgs::CameraInfoConstPtr cinfo_msg,
                                string target_frame,
                                image_transport::Publisher image_pub,
                                ros::Publisher image_raw_pub,
                                ros::Publisher cluster_pub,
                                ros::Publisher cloud_pub);
        
        void LocalCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& local_cloud);

        void inverseCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& inverse_cloud,
                          tf::Transform transform);

        void getClusterInfo(CloudA cloud,
                            Cluster& cluster);

        void clustering(CloudAPtr cloud,
                        vector<Clusters>& cluster_array);

        void iou(vector<Clusters> cluster_array,
                 sensor_msgs::CameraInfoConstPtr cinfo_msg,
                 CloudAPtr& cloud);

        void main();

        void loadPCDFile(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, string file_path);

        void CloudPublisher (CloudAPtr cloud, string target_frame, ros::Publisher pub);

        COLOR GetColor(double v, double vmin, double vmax);
};

DepthImage::DepthImage()
    : nh("~"), 
      it(nh), 
      obstacle_map(new CloudA),
      ground_map(new CloudA)
{
    nh.getParam("global_frame", global_frame);
    nh.getParam("laser_frame" , laser_frame);
    nh.getParam("zed0_frame"  , zed0_frame);
    nh.getParam("zed1_frame"  , zed1_frame);
    nh.getParam("zed2_frame"  , zed2_frame);

    nh.getParam("obstacle_path", OBSTACLE_PATH);
    nh.getParam("ground_path",GROUND_PATH);

    nh.getParam("threshold", threshold);

    nh.getParam("cell_size", cell_size);
    nh.getParam("grid_dimentions", grid_dimentions);
    nh.getParam("height_threshold", height_threshold);
    
    node_sub = nh.subscribe("/node", 10, &DepthImage::nodeCallback, this);

    zed0_pub = it.advertise("/zed0_depthimage", 10);
    zed1_pub = it.advertise("/zed1_depthimage", 10);
    zed2_pub = it.advertise("/zed2_depthimage", 10);
    
    zed0_raw_pub = nh.advertise<sensor_msgs::Image>("/zed0_raw", 10);
    zed1_raw_pub = nh.advertise<sensor_msgs::Image>("/zed1_raw", 10);
    zed2_raw_pub = nh.advertise<sensor_msgs::Image>("/zed2_raw", 10);

    zed0_rmground_pub = nh.advertise<sensor_msgs::PointCloud2>("/zed0_rmground", 10);
    zed1_rmground_pub = nh.advertise<sensor_msgs::PointCloud2>("/zed1_rmground", 10);
    zed2_rmground_pub = nh.advertise<sensor_msgs::PointCloud2>("/zed2_rmground", 10);

    zed0_cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("/zed0_cluster", 10);
    zed1_cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("/zed1_cluster", 10);
    zed2_cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("/zed2_cluster", 10);   
        
    zed0_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/zed0_reference_cloud", 10);
    zed1_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/zed1_reference_cloud", 10);
    zed2_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/zed2_reference_cloud", 10);
}


#include "depthimage_creater.cpp"

#endif
