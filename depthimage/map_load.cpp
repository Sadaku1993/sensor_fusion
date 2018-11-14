#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>

#include <boost/thread/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl_ros/point_cloud.h>

using namespace std;


string FILE_PATH = "/home/amsl/PCD/SQ2/20180707/Map/map.pcd";
// string FILE_PATH = "/home/amsl/PCD/Map/d_kan_around_si2017_gicp_ds.pcd";
string FRAME = "/map";

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
    nh.getParam("FILE_PATH", FILE_PATH);
    nh.getParam("FRAME", FRAME);

    ros::Rate rate(1);

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 10);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    loadPCDFile(cloud);

    //Downsample//
    pcl::VoxelGrid<pcl::PointXYZRGBNormal> vg;  
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ds_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);  
    vg.setInputCloud (cloud);  
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*ds_cloud);
    cout<<"----DownSampling:"<<ds_cloud->points.size()<<endl;

    while(ros::ok())
    {
        sensor_msgs::PointCloud2 pc2;
        pcl::toROSMsg(*ds_cloud, pc2);
        pc2.header.frame_id = FRAME;
        pc2.header.stamp = ros::Time::now();
        pub.publish(pc2);

        ros::spinOnce();
        rate.sleep();
    }

    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    // viewer = simpleVis(cloud);
    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce (100);
    //     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    // }

    return 0;
}
   
