/*
PointCloud2 to Image

Subscribe:
    pointcloud2
Publish:
    Image

author : Yudai Sadakuni
*/

#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_fusion/get_cluster_info.h>

#define HEIGHT 1200
#define WIDTH  800

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);

	Cluster cluster;
	getClusterInfo(*cloud, cluster);
	
	printf("Height:%.3f Width:%.3f\n", cluster.height, cluster.width);

	if( !cloud->empty() ){
        // Create cv::Mat
        cv::Mat image(HEIGHT, WIDTH, CV_8UC4 );

        int diff_row = WIDTH/2 - int(cluster.y*1000);
        int diff_col = HEIGHT/2 - int(cluster.z*1000);

        // pcl::PointCloud to cv::Mat
		for(size_t i=0;i<cloud->points.size();i++){
			int row = int( -(cloud->points[i].y - cluster.y - cluster.max_p[1])*1000 + diff_row);
			int col = int( -(cloud->points[i].z - cluster.z - cluster.max_p[2])*1000 + diff_col);
			cv::circle(image, 
					   cv::Point(row, col),
					   1,
					   cv::Scalar(255, 255, 255) 
					   ,-1);
		}
        cv::imshow("Image", image);
        cv::waitKey(1);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud2image");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cloud", 10, pcCallback);

    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

    
