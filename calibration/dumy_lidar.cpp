#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_fusion/get_cluster_info.h>

#define CELL_SIZE_H 0.016
#define CELL_SIZE_W 0.01
#define HEIGHT 50
#define WIDTH  1200

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

ros::Publisher pub;

void pc_Callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);
   
    CloudAPtr points(new CloudA);
    
    if(!cloud->empty()){
        Cluster cluster;
        getClusterInfo(*cloud, cluster);
		printf("Height:%.3f Width:%.3f\n", cluster.height, cluster.width);
		printf("centroid x:%.2f y:%.3f z:%.3f\n", cluster.x, cluster.y, cluster.z);

        for(int i=-WIDTH/2;i<WIDTH/2;i++){
            for(int j=-HEIGHT/2;j<HEIGHT/2;j++){
                PointA point;
                point.x = cluster.x;
                point.y = cluster.y + WIDTH*CELL_SIZE_W;
                point.z = cluster.z + HEIGHT*CELL_SIZE_H;
                points->points.push_back(point);
            }
        }
    }
	
    // Publish Coloured PointCloud
	sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*points, output);
    output.header.frame_id = msg->header.frame_id;
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dumy_lidar");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cloud", 10, pc_Callback);
    pub = n.advertise<sensor_msgs::PointCloud2>("/output", 10);

	ros::spin();

    return 0;
}

 
