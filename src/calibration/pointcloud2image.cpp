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
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.h>
#include <cv_bridge/cv_bridge.h>

typedef pcl::PointNormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);

    if( !cloud->empty() ){
    // Create cv::Mat
    image = cv::Mat( cloud->height, cloud->width, CV_8UC4 );

    // pcl::PointCloud to cv::Mat
    #pragma omp parallel for
    for( int y = 0; y < image.rows; y++ ) {
        for( int x = 0; x < image.cols; x++ ) {
            pcl::PointXYZRGBA point = cloud->at( x, y );
            image.at<cv::Vec4b>( y, x )[0] = point.b;
            image.at<cv::Vec4b>( y, x )[1] = point.g;
            image.at<cv::Vec4b>( y, x )[2] = point.r;
            image.at<cv::Vec4b>( y, x )[3] = point.a;
        }
    }
    cv::imshow("Image", image);
    cv::waitKey(1);
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

    
