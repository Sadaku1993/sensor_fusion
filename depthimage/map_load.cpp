#include <ros/ros.h>
#include <iostream>

#include <boost/thread/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

string FILE_PATH = "/home/amsl/PCD/SQ2/20180707/Map/map.pcd";

void loadPCDFile(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    cout<<"Load :" <<FILE_PATH<<endl;
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (FILE_PATH, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("-----Couldn't read file\n");
    }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGBNormal> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_load");

    ros::NodeHandle nh("~");

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    loadPCDFile(cloud);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    viewer = simpleVis(cloud);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}
   
