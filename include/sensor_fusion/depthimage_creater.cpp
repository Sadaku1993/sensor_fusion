void DepthImage::nodeCallback(const sensor_fusion::NodeConstPtr msg)
{
    cout<<"Node Callback"<<endl;

    cout<<"Transform Listener"<<endl;
    
    tf::Transform laser_transform;
    tf::Transform zed0_transform;
    tf::Transform zed1_transform;
    tf::Transform zed2_transform;

    tf::transformMsgToTF(msg->laser_transform, laser_transform);    // global_frame -- laser_frame
    tf::transformMsgToTF(msg->zed0_transform, zed0_transform);      // zed0_frame   -- laser_frame
    tf::transformMsgToTF(msg->zed1_transform, zed1_transform);      // zed1_frame   -- laser_frame
    tf::transformMsgToTF(msg->zed2_transform, zed2_transform);      // zed2_frame   -- laser_frame
    
    sensor_msgs::ImageConstPtr zed0_image (&msg->zed0_image); 
    sensor_msgs::ImageConstPtr zed1_image (&msg->zed1_image); 
    sensor_msgs::ImageConstPtr zed2_image (&msg->zed2_image); 

    sensor_msgs::CameraInfoConstPtr zed0_cinfo(&msg->zed0_cinfo); 
    sensor_msgs::CameraInfoConstPtr zed1_cinfo(&msg->zed1_cinfo);
    sensor_msgs::CameraInfoConstPtr zed2_cinfo(&msg->zed2_cinfo);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr zed0_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr zed1_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr zed2_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    // Transform PointCloud
}

void DepthImage::transform_pointcloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                                      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& trans_cloud,  
                                      tf::Transform transform,
                                      string target_frame,
                                      string source_frame)
{
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double z = transform.getOrigin().z();
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll ,pitch, yaw);

    cout<<setprecision(3)<<target_frame<<"-->"<<source_frame
        <<" x:"<<x<<" y:"<<y<<" z:"<<z<<" roll:"<<roll<<" pitch:"<<pitch<<" yaw:"<<yaw<<endl;

    pcl_ros::transformPointCloud(*cloud, *trans_cloud, transform);
    trans_cloud->header.frame_id = target_frame;
}

void DepthImage::depthimage_creater(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                                    sensor_msgs::ImageConstPtr image_msg,
                                    sensor_msgs::CameraInfoConstPtr cinfo_msg)
{
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
        cv_img_ptr = cv_bridge::toCvShare(image_msg);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    image = cv_bridge::toCvShare(image_msg)->image;

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cinfo_msg);

    for(pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator pt=cloud->points.begin(); pt<cloud->points.end(); pt++)
    {
        if ((*pt).x<0) continue;
        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);

        if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows){
            double range = sqrt( pow((*pt).x, 2.0) + pow((*pt).y, 2.0) + pow((*pt).z, 2.0));
            COLOR c = GetColor(int(range/20*255.0), 0, 255);
            cv::circle(image, uv, 3, cv::Scalar(int(255*c.b),int(255*c.g),int(255*c.r)), -1);
        }
    }

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	image_pub.publish(msg);
}
	
void DepthImage::loadPCDFile(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, string file_path)
{
    cout<<"Load :" <<file_path<<endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (file_path, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("-----Couldn't read file\n");
    }
}

COLOR DepthImage::GetColor(double v, double vmin, double vmax)
{
    COLOR c = {1.0, 1.0, 1.0}; // while
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

