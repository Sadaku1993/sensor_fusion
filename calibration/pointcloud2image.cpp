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

#define HEIGHT 800
#define WIDTH  1200
#define AREA 5

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;


void pcCallback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);
	
	// Create cv::Mat
	cv::Mat image = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
	int cols = image.cols;
	int rows = image.rows;
	for (int j = 0; j < rows; j++) {
		for (int i = 0; i < cols; i++) {
			image.at<cv::Vec3b>(j, i)[0] = 255; //青
			image.at<cv::Vec3b>(j, i)[1] = 255; //緑
			image.at<cv::Vec3b>(j, i)[2] = 255; //赤
		}
	}

	if( !cloud->empty() ){
		Cluster cluster;
		getClusterInfo(*cloud, cluster);

		printf("Size:%d\n", int(cloud->points.size()));
		printf("Height:%.3f Width:%.3f\n", cluster.height, cluster.width);
		printf("centroid y:%.3f z:%.3f\n", cluster.y, cluster.z);
		
		for(size_t i=0;i<cloud->points.size();i++){
			cloud->points[i].y -= cluster.y;
			cloud->points[i].z -= cluster.z;
		}
	
		// pcl::PointCloud to cv::Mat
		for(size_t i=0;i<cloud->points.size();i++){
			int col = int(cloud->points[i].z * 1000) + HEIGHT/2;
			int row = int(cloud->points[i].y * 1000) + WIDTH/2;
			
            image.at<cv::Vec3b>(col,row)[0] = 0; //青
            image.at<cv::Vec3b>(col,row)[1] = 0; //緑
            image.at<cv::Vec3b>(col,row)[2] = 0; //赤
        }
    }
	cv::namedWindow("Image");
	cv::imshow("Image", image);
	cv::waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud2image");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cloud", 10, pcCallback);

	ros::spin();

    return 0;
}

    
