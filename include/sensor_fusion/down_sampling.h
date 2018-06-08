#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

void down_sampling(CloudAPtr cloud, CloudAPtr& cloud_filtered)
{
    // Create the filtering object
    pcl::VoxelGrid<PointA> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.02f, 0.02f, 0.02f);
    sor.filter (*cloud_filtered);
}
