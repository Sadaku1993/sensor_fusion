#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <sensor_fusion/get_cluster_info.h>

#define CELL_SIZE_H 0.016
#define CELL_SIZE_W 0.01

#define SIZE 0.05

using namespace std;

ros::Publisher pub;
ros::Publisher pub_p1;

void pc_Callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);

    if(!cloud->empty())
    {
        Cluster cluster;
        getClusterInfo(*cloud, cluster);

        int size_height = int(cluster.height/CELL_SIZE_H);
        int size_width  = int(cluster.width/CELL_SIZE_W);

		printf("centroid x:%.2f y:%.3f z:%.3f\n", cluster.x, cluster.y, cluster.z);
		printf("Grid Size Height:%d Width:%d\n", size_height, size_width);

        /*       z
                 |
              P1 | P2
            y----------
              P3 | P4
                 |
        */
        PointA p0, p1, p2, p3, p4;
        p0.x = cluster.x; p0.y = cluster.y; p0.z = cluster.z; 
        p1.x = cluster.x; p1.y = cluster.y+0.15; p1.z = cluster.z+0.15;
        p2.x = cluster.x; p2.y = cluster.y-0.15; p2.z = cluster.z+0.15;
        p3.x = cluster.x; p3.y = cluster.y+0.15; p3.z = cluster.z-0.15;
        p4.x = cluster.x; p4.y = cluster.y-0.15; p4.z = cluster.z-0.15;

        CloudAPtr cloud_p1(new CloudA);
        CloudAPtr cloud_p2(new CloudA);
        CloudAPtr cloud_p3(new CloudA);
        CloudAPtr cloud_p4(new CloudA);

        for(int i=0;i<100;i++)
        {
            PointA searchPoint;
            searchPoint.x = p1.x + 0.0001 * (rand()%1024 - 512);
            searchPoint.y = p1.y + 0.0001 * (rand()%1024 - 512);
            searchPoint.z = p1.z + 0.0001 * (rand()%1024 - 512);
            cloud_p1->points.push_back(searchPoint);
        }

        CloudAPtr centroid(new CloudA);
        centroid->points.push_back(p0);
        centroid->points.push_back(p1);
        centroid->points.push_back(p2);
        centroid->points.push_back(p3);
        centroid->points.push_back(p4);

		// Publish Coloured PointCloud
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(*centroid, output);
		output.header.frame_id = msg->header.frame_id;
		output.header.stamp = ros::Time::now();
		pub.publish(output);

		sensor_msgs::PointCloud2 output_p1;
		pcl::toROSMsg(*cloud_p1, output_p1);
		output_p1.header.frame_id = msg->header.frame_id;
		output_p1.header.stamp = ros::Time::now();
		pub_p1.publish(output_p1);

    }
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "lidar_seg_circle");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cloud", 10, pc_Callback);
    pub = n.advertise<sensor_msgs::PointCloud2>("/output", 10);
    pub_p1 = n.advertise<sensor_msgs::PointCloud2>("/output/p1", 10);

    ros::spin();

    return 0;
}
