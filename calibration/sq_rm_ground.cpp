#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define cell_size 0.1
#define grid_dimentions 60

#define threshold 0.20
#define max_threshold 1.0
#define min_threshold -1.0

ros::Publisher pub;

using namespace std;

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

void constructFullClouds(CloudAPtr cloud, CloudA& rm_ground){
// #pragma omp parallel for
    for(size_t i=0;i<cloud->points.size();i++)
    {
        int x = (grid_dimentions/2)+cloud->points[i].x/cell_size;
        int y = (grid_dimentions/2)+cloud->points[i].y/cell_size;

        if(0<=x && x<grid_dimentions &&
           0<=y && y<grid_dimentions)
        {
			if(min_threshold<cloud->points[i].z &&
			   cloud->points[i].z < max_threshold)
 	           rm_ground.points.push_back(cloud->points[i]);
        }
    }
}
                         

void Callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);
	
	CloudA rm_ground;
	constructFullClouds(cloud, rm_ground);

	// Publish Coloured PointCloud
	sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(rm_ground, output);
    output.header.frame_id = msg->header.frame_id;
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "sq_rm_ground");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cloud", 10, Callback);
    pub = n.advertise<sensor_msgs::PointCloud2>("/rm_ground", 10);
	
    ros::spin();

    return 0;
}
