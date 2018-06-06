#include <ros/ros.h>
#include "ros/package.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

// Callback(Image)
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // cv_bridge
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
        cv_img_ptr = cv_bridge::toCvShare(msg);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    image = cv_bridge::toCvShare(msg)->image;
    
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
                     50,   // param2 
                     20,   // minRadius
                     100   // maxRadius
					 );

    // Show results
    for(vector<cv::Vec3f>::iterator it = circles.begin(); it!=circles.end(); ++it)
    {
        cv::Point center = cv::Point((*it)[0], (*it)[1]);
        int radius = (*it)[2];
        cv::circle(image, center, radius, cv::Scalar(0, 0, 255), 2);
    }
    cv::namedWindow("Gray-HoughCircles");
    cv::imshow("Gray-HoughCircles", image);

    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depthimage");
    ros::NodeHandle n;

    ros::Subscriber image_sub = n.subscribe("/image", 10, imageCallback);

    ros::spin();

    return 0;
}
