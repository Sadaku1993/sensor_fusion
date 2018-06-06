#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int SAVE_NUM;
int LIDAR_POINTS;
int HZ;

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;

ros::Publisher pub;

int pc_count = 0;
int t = 0;
bool save = false;

CloudA pcl_save;

void Callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);

    printf("count:%d size:%d\n", pc_count, int(cloud->points.size()));
    t = pc_count * int(cloud->points.size());

    for(size_t i=0;i<LIDAR_POINTS;i++)
    {
        pcl_save.points[i+t] = cloud->points[i];
    }

    // Publish Coloured PointCloud
    if(save && pc_count%HZ==0)
    {
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(pcl_save, output);
        output.header.frame_id = msg->header.frame_id;
        output.header.stamp = ros::Time::now();
        pub.publish(output);
    }

    ++pc_count;
    if(pc_count == SAVE_NUM){
        save = true;
        pc_count = 0;
    }
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "save_points");
    ros::NodeHandle n;

	n.getParam("save_points/save_num", SAVE_NUM);
	n.getParam("save_points/lidar_points", LIDAR_POINTS);
	n.getParam("save_points/hz" ,HZ);

    ros::Subscriber sub = n.subscribe("/cloud", 10, Callback);
    pub = n.advertise<sensor_msgs::PointCloud2>("/cloud/save", 10);

    pcl_save.points.resize(SAVE_NUM*LIDAR_POINTS);
	
    ros::spin();

    return 0;
}
