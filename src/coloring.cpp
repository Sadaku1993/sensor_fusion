/*
Coloring PointCloud using Sensor Fusion

Subscribe:
    camera_image
    camera_info
    Lidar_pointcloud
    tf
Publish:
    colored_pointcloud

author : Yudai Sadakuni
 
*/

#include <sensor_fusion/coloring.h>

// Colouring Function
void colouring(sensor_msgs::PointCloud2 pc_msg, const sensor_msgs::CameraInfoConstPtr& cinfo_msg, const sensor_msgs::ImageConstPtr& image_msg)
{
    cout<<"ALL GREEN"<<endl;

    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
      cv_img_ptr = cv_bridge::toCvShare(image_msg);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    image = cv_bridge::toCvShare(image_msg)->image;

    image_geometry::PinholeCameraModel cam_model_;
    cam_model_.fromCameraInfo(cinfo_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>); // From ROS Msg
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>); // After transformation
    PointCloudXYZRGB::Ptr coloured = PointCloudXYZRGB::Ptr(new PointCloudXYZRGB); // For coloring purposes
    fromROSMsg(pc_msg, *trans_cloud);

    trans_cloud->header.frame_id = TARGET_FRAME;

    pcl::copyPointCloud(*trans_cloud, *coloured);
 	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr area(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pt = coloured->points.begin(); pt < coloured->points.end(); pt++)
    {
        if ((*pt).x<0) continue;
        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model_.project3dToPixel(pt_cv);

        if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows){
            (*pt).b = image.at<cv::Vec3b>(uv)[0];
            (*pt).g = image.at<cv::Vec3b>(uv)[1];
            (*pt).r = image.at<cv::Vec3b>(uv)[2];
            area->points.push_back(*pt);
        }
        else{
            (*pt).b = 255;
            (*pt).g = 255;
            (*pt).r = 255;
        }
    }
    ROS_INFO("Publish coloured PC");

    // Publish Coloured PointCloud
    sensor_msgs::PointCloud2 pcl_coloured;
    pcl::toROSMsg(*area, pcl_coloured);
    pcl_coloured.header.frame_id = TARGET_FRAME;
    pcl_coloured.header.stamp = t;
    pub.publish(pcl_coloured);
    
    pc_flag = false;
    camera_flag = false;
    image_flag = false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coloring");
    ros::NodeHandle n;

    // get param from yaml file
    n.getParam("/zed_coloring/laser_topic", LASER_TOPIC);
    n.getParam("/zed_coloring/camera_info_topic", CAMERA_INFO_TOPIC);
    n.getParam("/zed_coloring/image_topic", IMAGE_TOPIC);
    n.getParam("/zed_coloring/target_frame", TARGET_FRAME);
    n.getParam("/zed_coloring/source_frame", SOURCE_FRAME);
    n.getParam("/zed_coloring/publish_topic", PUBLISH_TOPIC);

    // define tf
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    // define pub sub data
    ros::Subscriber pc_sub = n.subscribe("LIDAR_TOPIC",1,pcCallback);
    ros::Subscriber cinfo_sub = n.subscribe("CAMERA_INFO_TOPIC",1,cameraCallback);
    ros::Subscriber image_sub = n.subscribe("IMAGE_TOPIC",1,imageCallback);
    pub = n.advertise<sensor_msgs::PointCloud2>("PUBLISH_TOPIC",10);

    ros::Rate rate(20);

    while(ros::ok()){
        // Transform pointcloud 
        sensor_msgs::PointCloud pc_trans;
        sensor_msgs::PointCloud2 pc2_trans;
        
        try{
            listener.waitForTransform(TARGET_FRAME, SOURCE_FRAME, t, ros::Duration(1.0));
            listener.transformPointCloud(TARGET_FRAME t, pc_tmp, SOURCE_FRAME, pc_trans);
            sensor_msgs::convertPointCloudToPointCloud2(pc_trans, pc2_trans);
        }catch (tf::TransformException& ex) {
            ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        }

        // If recieve all topics run coloring function
        if (pc_flag && camera_flag && image_flag) coloring(pc2_trans, camera_, image_);
        ros::spinOnce();
        rate.sleep();
    }       

    return 0;
}
