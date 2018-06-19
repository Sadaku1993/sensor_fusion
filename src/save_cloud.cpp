/*
Save PointCloud Data

Param:
    target_frame
    source_frame

author:
    Yudai Sadakuni

*/

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <sys/stat.h>
#include <sys/types.h>

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;

string FILE_PATH="~/PCD/Sensor_Fusion";

string TARGET_FRAME;
string SOURCE_FRAME;

int COUNT = 0;

CloudAPtr input_ (new CloudA);
void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, *input_);
}

void savePCDFile(CloudAPtr cloud, 
                 int count)
{
    string file_name = to_string(count);
    pcl::io::savePCDFileASCII(FILE_PATH+file_name+".pcd", *cloud);
    printf("saved %d\n", int(cloud->points.size()));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_cloud");
    ros::NodeHandle n;
    
    ros::Rate rate(20);

    n.getParam("target_frame", TARGET_FRAME);
    n.getParam("source_frame", SOURCE_FRAME);

    tf::TransformListener listener;

    ros::Subscriber sub = n.subscribe("/cloud", 10, pcCallback);
    
    if(mkdir(FILE_PATH.c_str()，0755) == 0) {
        printf("Create Folder\n");
    }
    else {
        printf("Fail to Create Folder\n");
    }
    
    while(ros::ok())
    {
        try{   
            listener.waitForTransform(TARGET_FRAME, SOURCE_FRAME, 
                                      ros::Time(0), ros::Duration(1.0));
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        CloudAPtr cloud_tf(new CloudA);
        pcl_ros::transformPointCloud(TARGET_FRAME,
                *input_, 
                *cloud_tf, 
                listener);
            
        savePCDFile(cloud_tf, COUNT);
        COUNT += 1;
    }
    return 0;
}
