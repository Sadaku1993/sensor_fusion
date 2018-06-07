#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_fusion/get_cluster_info.h>

#define CELL_SIZE_H 0.016
#define CELL_SIZE_W 0.01


ros::Publisher pub;

void pc_Callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);
   
    
    if(!cloud->empty()){
		CloudAPtr points;
		Cluster cluster;
        getClusterInfo(*cloud, cluster);
		printf("Height:%.3f Width:%.3f\n", cluster.height, cluster.width);
		printf("centroid x:%.2f y:%.3f z:%.3f\n", cluster.x, cluster.y, cluster.z);
		
		int size_height = int(cluster.height/CELL_SIZE_H);
		int size_width = int(cluster.width/CELL_SIZE_W);

		printf("Points Height:%d Width:%d\n", size_height, size_width);

		int min_height = -size_height/2;
		int max_height = size_height/2;
		int min_width  = -size_width/2;
		int max_width  = size_width/2;
		
		for(int i=min_width;i<max_width;i++){
			for(int j=min_height;j<max_height;j++){
				PointA p;
				p.x = cluster.x;
				p.y = cluster.y; // + i*CELL_SIZE_W;
				p.z = cluster.z; // + j*CELL_SIZE_H;
				points->points.push_back(p);
             }
         }
		 // Publish Coloured PointCloud
		 sensor_msgs::PointCloud2 output;
		 pcl::toROSMsg(*points, output);
		 output.header.frame_id = msg->header.frame_id;
		 output.header.stamp = ros::Time::now();
		 pub.publish(output);

	}

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

 
