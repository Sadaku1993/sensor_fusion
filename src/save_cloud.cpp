/*
Save PointCloud Data

Param:
    target_frame
    source_frame

author:
    Yudai Sadakuni

warning:
    prease check your file path

    home
      Lamsl
        LPCD
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
#include <iostream>

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;

string HOME_DIRS="/home/amsl";
string FILE_PATH="PCD";
string SAVE_PATH="Save";

string TARGET_FRAME;
string SOURCE_FRAME;

int COUNT = 0;

bool flag = false;
CloudAPtr input_ (new CloudA);
void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, *input_);
    SOURCE_FRAME = msg->header.frame_id;
    flag = true;
}

void savePCDFile(CloudAPtr cloud, 
                 int count)
{
    string file_name = to_string(count);
    string path = HOME_DIRS + "/" + FILE_PATH + "/" + SAVE_PATH;
    pcl::io::savePCDFileASCII(path+"/"+file_name+".pcd", *cloud);
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
    
    // move to home dirs and create pcd folder
    chdir(HOME_DIRS.c_str());
    if(mkdir(FILE_PATH.c_str(), 0755) == 0)
    {
        printf("Create PCD Folder\n");
    }else{
        printf("Fail to Create Folder\n");
    }
    chdir(FILE_PATH.c_str());
    if(mkdir(SAVE_PATH.c_str(), 0755) == 0)
    {
        printf("Create SAVE Folder\n");
    }else{
        printf("Fail to Create Folder\n");
    }


    while(ros::ok())
    {
        CloudAPtr cloud_tf(new CloudA);
        
        if(flag){
            try{   
                listener.waitForTransform(TARGET_FRAME, SOURCE_FRAME, 
                        ros::Time(0), ros::Duration(1.0));
                pcl_ros::transformPointCloud(TARGET_FRAME, *input_, 
                        *cloud_tf, listener);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

            if(0<int(cloud_tf->points.size())){
                savePCDFile(cloud_tf, COUNT);
                COUNT += 1;
            }
        }
    }
    return 0;
}
