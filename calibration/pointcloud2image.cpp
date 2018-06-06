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

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);
	
	// Create cv::Mat
	cv::Mat image = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

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
			int row = int(cloud->points[i].y * 1000) + WIDTH/2;
			int col = int(cloud->points[i].z * 1000) + HEIGHT/2;

			cv::circle(image, cv::Point(row, col), 3, cv::Scalar(255, 255, 255), -1, CV_AA);
		}

		// Gray Scale
		cv::Mat gray_image;
		cv::cvtColor(image, gray_image, CV_BGR2GRAY);

		// GaussianBlur
		cv::GaussianBlur(gray_image, gray_image, cv::Size(11, 11), 2, 2);

		// Hough Circles
		vector<cv::Vec3f> circles;
		cv::HoughCircles(gray_image, 
				circles, 
				CV_HOUGH_GRADIENT, 
				1,    // 画像分解能に対する投票分解能の比率の逆数
				100,  // 検出される円の中心同士の最小距離
				20,   // param1
				40,   // param2 
				50,   // minRadius
				90   // maxRadius
				);

		// Show results
		for(vector<cv::Vec3f>::iterator it = circles.begin(); it!=circles.end(); ++it)
		{
			cv::Point center = cv::Point((*it)[0], (*it)[1]);
			int radius = (*it)[2];
			cv::circle(image, center, radius, cv::Scalar(0, 0, 255), 2);
		}
	}
	cv::namedWindow("Gray-HoughCircles");
	cv::imshow("Gray-HoughCircles", image);
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

    
