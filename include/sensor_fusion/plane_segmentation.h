#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace Eigen;

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

void plane_segmentation(CloudAPtr cloud, CloudAPtr& plane, float distance)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distance);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }
    
    // std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
    //     << coefficients->values[1] << " "
    //     << coefficients->values[2] << " " 
    //     << coefficients->values[3] << std::endl;
    
    // plane pointcloud
    plane->points.resize(inliers->indices.size());
    for(size_t i=0;i<inliers->indices.size();i++)
    {
        plane->points[i] = cloud->points[inliers->indices[i]];
    }
}
