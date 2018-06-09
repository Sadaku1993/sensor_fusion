/* 
 * author : Yudai Sadakuni
 *
 * plane segmentation 
 *
 */


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <sensor_fusion/clustering.h>
#include <sensor_fusion/plane_segmentation.h>
#include <sensor_fusion/outlier_removal.h>
#include <sensor_fusion/down_sampling.h>
#include <sensor_fusion/pub_cloud.h>

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;

ros::Publisher pub_area;
ros::Publisher pub_down_sample;
ros::Publisher pub_plane;
ros::Publisher pub_filtered;


// このパラメータで取得する点群のエリアを確定する
// キャリブレーションボードが収まる範囲に調整すること
double MIN_X, MAX_X;
double MIN_Y, MAX_Y;
double MIN_Z, MAX_Z;

// 正面の点群のみを取得
void pickup_cloud(CloudAPtr cloud, CloudAPtr& output)
{
    for(size_t i=0;i<cloud->points.size();i++){
        if( MIN_X < cloud->points[i].x && cloud->points[i].x < MAX_X
            && MIN_Y < cloud->points[i].y && cloud->points[i].y < MAX_Y
            && MIN_Z < cloud->points[i].z && cloud->points[i].z < MAX_Z)
            
            output->points.push_back(cloud->points[i]);
    }
}

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    CloudAPtr input (new CloudA);
    pcl::fromROSMsg(*msg, *input);

    // Pick Up Cloud
    CloudAPtr area(new CloudA);
    if(0<input->points.size())
        pickup_cloud(input, area);

    // Down Sampling
    // CloudAPtr ds_cloud(new CloudA);
	// if(0<area->points.size())
    //     down_sampling(area, ds_cloud);

    // Clustering
    // vector<Clusters> cluster_array;
    // if(0<ds_cloud->points.size())
    //     clustering(ds_cloud, cluster_array);

	// Plane Segmentation
    CloudAPtr plane(new CloudA);
    if(0<area->points.size())
		plane_segmentation(area, plane);

    // Outlier Removal
    CloudAPtr filtered(new CloudA);
    if(0<plane->points.size())
        outlier_removal(plane, filtered);

    pub_cloud(area, msg->header, pub_area);
    // pub_cloud(ds_cloud, msg->header, pub_down_sample);
    pub_cloud(plane, msg->header, pub_plane);
    pub_cloud(filtered, msg->header, pub_filtered);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ransac_lidar");
    ros::NodeHandle n;

    n.getParam("min_x", MIN_X);
    n.getParam("max_x", MAX_X);
    n.getParam("min_y", MIN_Y);
    n.getParam("max_y", MAX_Y);
    n.getParam("min_z", MIN_Z);
    n.getParam("max_z", MAX_Z);

    ros::Subscriber sub = n.subscribe("/cloud", 10, pcCallback);

    pub_area        = n.advertise<sensor_msgs::PointCloud2>("/area", 10);
    pub_down_sample = n.advertise<sensor_msgs::PointCloud2>("/down_sample", 10);
    pub_plane       = n.advertise<sensor_msgs::PointCloud2>("/plane", 10);
    pub_filtered     = n.advertise<sensor_msgs::PointCloud2>("/filtered", 10);

	ros::spin();

    return 0;
}
