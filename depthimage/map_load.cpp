#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>

#include <boost/thread/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl_ros/point_cloud.h>

using namespace std;


// string FILE_PATH = "/home/amsl/PCD/SQ2/20180707/Map/map.pcd";
string FILE_PATH = "/home/amsl/PCD/Map/d_kan_around_si2017_gicp_ds.pcd";
string FRAME = "/map";

void loadPCDFile(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
{
    cout<<"Load :" <<FILE_PATH<<endl;
    
    if (pcl::io::loadPCDFile<pcl::PointXYZINormal> (FILE_PATH, *cloud) == -1) //* load the file
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
    ros::Rate rate(1);

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 10);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    loadPCDFile(cloud);

    

    while(ros::ok())
    {
        sensor_msgs::PointCloud2 pc2;
        pcl::toROSMsg(*cloud, pc2);
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
   
