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

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;

string TARGET_FRAME;
string SOURCE_FRAME;

int COUNT = 0;

CloudAPtr input_ (new CloudA);
void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, *input_);
}

void savePCDFile(CloudAPtr cloud, 
                 tf::TransformListener listener, 
                 string file_name)
{
    CloudAPtr cloud_out(new CloudA);

    pcl_ros::transformPointCloud(TARGET_FRAME, 
                                 *cloud, 
                                 *cloud_out, 
                                 listener);
    pcl::io::savePCDFIleASCII(file_name+".pcd", *cloud_out);
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

    while(ros::ok())
    {
        listener.waitForTransform(TARGET_FRAME, SOURCE_FRAME, ros::Time(0), ros::Duration(1.0));
        file_name=to_string(COUNT);
        savePCDFile(input_, listener, file_name);
        COUNT += 1;
    }
    return 0;
}
