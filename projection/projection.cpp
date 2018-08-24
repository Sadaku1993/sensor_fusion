/*
    Coloring PointCloud (SQ_LiDAR and ZED)

    author : Yudai Sadakuni
*/

#include <ros/ros.h>
#include "ros/package.h"

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

image_transport::Publisher image_pub;
ros::Publisher cloud_pub;

typedef struct{
    double r, g, b;
} COLOUR;

// gradation depth data
COLOUR GetColour(double v, double vmin, double vmax)
{
    COLOUR c = {1.0, 1.0, 1.0}; // while
    double dv;

   if (v < vmin)
   v = vmin;
   if (v > vmax)
   v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
      c.r = 0;
      c.g = 4 * (v - vmin) / dv;
   } else if (v < (vmin + 0.5 * dv)) {
      c.r = 0;
      c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
   } else if (v < (vmin + 0.75 * dv)) {
      c.r = 4 * (v - vmin - 0.5 * dv) / dv;
      c.b = 0;
   } else {
      c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
      c.b = 0;
   }

   return(c);
}

PointCloud2 pc_msg;
void pc_callback(const PointCloud2ConstPtr msg)
{
    pc_msg = *msg;
}


void callback(const ImageConstPtr& image_msg,
              const CameraInfoConstPtr& cinfo_msg)
{
    cout<<"ALL GREEN"<<endl;
    
    // cv_bridge
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
      cv_img_ptr = cv_bridge::toCvShare(image_msg);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    image = cv_bridge::toCvShare(image_msg)->image;

	cv::Mat image_copy = image.clone();
    
    // camera info
    image_geometry::PinholeCameraModel cam_model_;
    cam_model_.fromCameraInfo(cinfo_msg);

    // Coloring Step
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr area(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(pc_msg, *cloud);
	// cloud->header.frame_id = pc_msg.header.frame_id;
	
    for(pcl::PointCloud<pcl::PointXYZ>::iterator pt = cloud->points.begin(); pt < cloud->points.end(); pt++)
    {
        if ((*pt).x<0) continue;
        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model_.project3dToPixel(pt_cv);

        if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows)
        {
			// PointCloud
            pcl::PointXYZRGB p;
            p.x = (*pt).x;
            p.y = (*pt).y;
            p.z = (*pt).z;
            p.b = image.at<cv::Vec3b>(uv)[0];
            p.g = image.at<cv::Vec3b>(uv)[1];
            p.r = image.at<cv::Vec3b>(uv)[2];
            area->points.push_back(p);
			
			//Image
			double range = sqrt( pow((*pt).x, 2.0) + pow((*pt).y, 2.0) + pow((*pt).z, 2.0));
			COLOUR c = GetColour(int(range/20*255.0), 0, 255);
			cv::circle(image_copy, uv, 3, cv::Scalar(int(255*c.b),int(255*c.g),int(255*c.r)), -1);
		}
	}
	// Publish PointCloud
    PointCloud2 output;
    pcl::toROSMsg(*area, output);
    output.header.frame_id = pc_msg.header.frame_id;
    output.header.stamp = ros::Time::now();
    cloud_pub.publish(output);
	// Publish Image
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_copy).toImageMsg();
	image_pub.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coloring");
    ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);

    ros::Subscriber cloud_sub = nh.subscribe("/cloud", 10, pc_callback);
	message_filters::Subscriber<Image> image_sub(nh, "/image", 10);
	message_filters::Subscriber<CameraInfo> cinfo_sub(nh, "/camera_info", 10);

	typedef sync_policies::ApproximateTime<Image, CameraInfo> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, cinfo_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/output", 10);
	image_pub = it.advertise("/image/output", 10);

    ros::spin();

    return 0;
}
