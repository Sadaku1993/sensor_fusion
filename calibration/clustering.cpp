#include <ros/ros.h>
#include "ros/package.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>


typedef pcl::PointNormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;
using namespace Eigen;

struct Cluster{
    float x;
    float y;
    float z;

    float width;
    float height;
    float depth;

    float curvature;

};

string FRAME;

ros::Publisher pub;

void clustering(CloudAPtr cloud, CloudA cluster)
{
    // downsampled point's z =>0
    vector<float> tmp_z;
    tmp_z.resize(cloud_in->points.size());
	for(int i=0;i<(int)cloud_in->points.size();i++){
        tmp_z[i]=cloud_in->points[i].z;
		cloud_in->points[i].z  = 0.0;
    }
    //Clustering//
    pcl::search::KdTree<PointA>::Ptr tree (new pcl::search::KdTree<PointA>);
    tree->setInputCloud (cloud_in);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointA> ec;
    ec.setClusterTolerance (0.05); // 15cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (10000);
    ec.setSearchMethod (tree);
    ec.setInputCloud(cloud_in);
    ec.extract (cluster_indices);
    //reset z value
	for(int i=0;i<(int)cloud_in->points.size();i++)
        cloud_in->points[i].z=tmp_z[i];

    for(int iii=0;iii<(int)cluster_indices.size();iii++)
    {
        // cluster points
        CloudAPtr cloud_cluster (new CloudA);
        cloud_cluster->points.resize(cluster_indices[iii].indices.size());
        // cluster data
        Cluster data;
        for(int jjj=0;jjj<int(cluster_indices[iii].indices.size());jjj++){
            int p_num = cluster_indices[iii].indices[jjj];
            cloud_cluster->points[jjj] = cloud_in->points[p_num];
        }
        getClusterInfo(*cloud_cluster, data);
    }
}


void pcCallback(const sensor_msgs::PointCloudConstPtr& msg)
{
    CloudAPtr cloud (new CloudA);
    pcl::fromROSMsg(*msg, *cloud);

    CloudA cluster;
    clustering(cloud, cluster);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cluster, output);
    output.header.stamp = ros::Time::now();
    output.header.frame_id = msg->header.frame_id;
    pub.publish(output);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "clustering");
    ros::NodeHandle n;

    n.getParam("frame", FRAME);

    ros::Subscriber sub = n.subscribe("/cloud", 10, pcCallback);
    pub = n.advertise<sensor_msgs::PointCloud2>("/cluster", 1);

    ros::Rate rate(10);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
