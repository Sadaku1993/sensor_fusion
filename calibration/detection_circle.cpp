#include <ros/ros.h>
#include "ros/package.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

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
    image = cv_bridge::toCvShare(image_msg)->image;
    
    // GaussianBlur
    cv::GaussianBlur(image, image, cv::Size(5, 5), 0);
    cv::namedWindow("Blur Image");
    cv::imshow("Blur Image", image);

    //Gray Scale
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::CV_BGR2GRAY);
    cv::namedWindow("Blur-Gray Image");
    cv::imshow("Blur-Gray Image", gray_image);

    //Hough Circles
    cv::HoughCircles(gray_image, circles, cv::CV_HOUGH_GRADIENT, 1, 50, 100, 100, 0, 0);
    cv::Mat copy = image.clone();

    for(auto it = circles.begin()


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depthimage");
    ros::NodeHandle n;

    ros::Subscriber image_sub = n.subscribe("/image", 10, imageCallback);

    ros::spin();

    return 0;
}
