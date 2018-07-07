void DepthImage::nodeCallback(const sensor_fusion::NodeConstPtr msg)
{
    cout<<"Node:"<<msg->node<<endl;

    tf::Transform laser_transform;
    tf::Transform zed0_transform;
    tf::Transform zed1_transform;
    tf::Transform zed2_transform;

    tf::transformMsgToTF(msg->laser_transform, laser_transform);    // global_frame      -- laser_frame
    tf::transformMsgToTF(msg->zed0_transform, zed0_transform);      // zed0_left_frame   -- laser_frame
    tf::transformMsgToTF(msg->zed1_transform, zed1_transform);      // zed1_left_frame   -- laser_frame
    tf::transformMsgToTF(msg->zed2_transform, zed2_transform);      // zed2_left_frame   -- laser_frame
    
    sensor_msgs::Image::Ptr zed0_image(new sensor_msgs::Image);
    sensor_msgs::Image::Ptr zed1_image(new sensor_msgs::Image);
    sensor_msgs::Image::Ptr zed2_image(new sensor_msgs::Image);

    sensor_msgs::CameraInfo::Ptr zed0_cinfo(new sensor_msgs::CameraInfo);
    sensor_msgs::CameraInfo::Ptr zed1_cinfo(new sensor_msgs::CameraInfo);
    sensor_msgs::CameraInfo::Ptr zed2_cinfo(new sensor_msgs::CameraInfo);

    *zed0_image = msg->zed0_image;
    *zed1_image = msg->zed1_image;
    *zed2_image = msg->zed2_image;

    *zed0_cinfo = msg->zed0_cinfo;
    *zed1_cinfo = msg->zed1_cinfo;
    *zed2_cinfo = msg->zed2_cinfo;

    // sensor_msgs::Image::Ptr zed0_image = boost::shared_ptr<sensor_msgs::Image>(msg->zed0_image);
    // sensor_msgs::Image::Ptr zed1_image = boost::shared_ptr<sensor_msgs::Image>(msg->zed1_image);
    // sensor_msgs::Image::Ptr zed2_image = boost::shared_ptr<sensor_msgs::Image>(msg->zed2_image);

    // sensor_msgs::CameraInfo::Ptr zed0_cinfo = boost::shared_ptr<sensor_msgs::CameraInfo>(msg->zed0_cinfo);
    // sensor_msgs::CameraInfo::Ptr zed1_cinfo = boost::shared_ptr<sensor_msgs::CameraInfo>(msg->zed1_cinfo);
    // sensor_msgs::CameraInfo::Ptr zed2_cinfo = boost::shared_ptr<sensor_msgs::CameraInfo>(msg->zed2_cinfo);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr local_map(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pickup_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr zed0_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr zed1_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr zed2_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    // 保存したMapの座標系はGlobal(Map)座標系になっている
    // laser_transform(global -- laser)の逆行列を使って laser座標系に変換する
    inverseCloud(map, local_map, laser_transform);

    // Laser座標系を中心としてthreshold以内の点群をMapから取得
    LocalCloud(local_map, pickup_cloud);
    sensor_msgs::PointCloud2 local_map_pc2;
    pcl::toROSMsg(*pickup_cloud, local_map_pc2);
    local_map_pc2.header.frame_id = laser_frame;
    local_map_pc2.header.stamp = ros::Time::now();
    local_map_pub.publish(local_map_pc2);

    // laser座標系に変換したMapデータをカメラ座標に変換
    transform_pointcloud(pickup_cloud, zed0_cloud, zed0_transform, zed0_frame, laser_frame);
    transform_pointcloud(pickup_cloud, zed1_cloud, zed1_transform, zed1_frame, laser_frame);
    transform_pointcloud(pickup_cloud, zed2_cloud, zed2_transform, zed2_frame, laser_frame);

    // DepthImageを作成
    depthimage_creater(zed0_cloud, zed0_image, zed0_cinfo, zed0_pub, zed0_raw_pub);
    depthimage_creater(zed1_cloud, zed1_image, zed1_cinfo, zed1_pub, zed1_raw_pub);
    depthimage_creater(zed2_cloud, zed2_image, zed2_cinfo, zed2_pub, zed2_raw_pub);

    cout<<"----Finish"<<endl;

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

    cout<<setprecision(3)<<"----"<<target_frame<<"-->"<<source_frame
        <<" x:"<<x<<" y:"<<y<<" z:"<<z<<" roll:"<<roll<<" pitch:"<<pitch<<" yaw:"<<yaw<<endl;

    pcl_ros::transformPointCloud(*cloud, *trans_cloud, transform);
    trans_cloud->header.frame_id = target_frame;
}

void DepthImage::depthimage_creater(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                                    sensor_msgs::ImageConstPtr image_msg,
                                    sensor_msgs::CameraInfoConstPtr cinfo_msg,
                                    image_transport::Publisher image_pub,
                                    ros::Publisher image_raw_pub)
{
    // Visualize用
    sensor_msgs::Image tmp_image = *image_msg;

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


//     double distance[cv_img_ptr->image.rows][cv_img_ptr->image.cols];
//     memset(&distance, threshold, cv_img_ptr->image.rows*cv_img_ptr->image.cols);
// 
//     for(pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator pt=cloud->points.begin(); pt<cloud->points.end(); pt++)
//     {
//         if ((*pt).x<0) continue;
//         cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
//         cv::Point2d uv;
//         uv = cam_model.project3dToPixel(pt_cv);
// 
//         if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows){
//             double range = sqrt( pow((*pt).x, 2.0) + pow((*pt).y, 2.0) + pow((*pt).z, 2.0));
//             if(range<threshold){
//                 distance[int(uv.y)][int(uv.x)] = range;
//             }
//         }
//     }
//     
//     for(int y=0; y<image.rows; y++){
//         for(int x=0; x<image.cols; x++){
//             double range = distance[y][x];
//             if(0.01<range && range<threshold){
//                 COLOR c = GetColor(int(range/50*255.0), 0, 255);
//                 cv::Point2d uv;
//                 uv.x = x;
//                 uv.y = y;
//                 cv::circle(image, uv, 3, cv::Scalar(int(255*c.b),int(255*c.g),int(255*c.r)), -1);
//             }
//         }
//     }
//     
// 
    cv::Mat gray_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, CV_8UC1);
    for(int y=0; y<gray_image.rows; y++){
        for(int x=0; x<gray_image.cols; x++){
            gray_image.at<unsigned char>(y, x) = 0.0;
        }
    }
    
    for(pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator pt=cloud->points.begin(); pt<cloud->points.end(); pt++)
    {
        if ((*pt).x<0) continue;
        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);

        if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows){
            double range = sqrt( pow((*pt).x, 2.0) + pow((*pt).y, 2.0) + pow((*pt).z, 2.0));

            if(range<threshold){
                gray_image.at<unsigned char>(int(uv.y), int(uv.x)) = range;
                cv::circle(gray_image, uv, 3, cv::Scalar(range, range, range) , -1);
            }
        }
    }

#pragma omp parallel for
    for(int y=0; y<gray_image.rows; y++){
        for(int x=0; x<gray_image.cols; x++){
            double range = gray_image.at<unsigned char>(y, x);
            if(0.01<range && range<threshold){
                COLOR c = GetColor(int(range/50*255.0), 0, 255);
                cv::Point2d uv;
                uv.x = x;
                uv.y = y;
                cv::circle(image, uv, 3, cv::Scalar(int(255*c.b),int(255*c.g),int(255*c.r)), -1);
                // image.at<cv::Vec3b>(y, x)[0] = 255*c.b;
                // image.at<cv::Vec3b>(y, x)[1] = 255*c.g;
                // image.at<cv::Vec3b>(y, x)[2] = 255*c.r;
            }
        }
    }

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	image_pub.publish(msg);

	image_raw_pub.publish(tmp_image);
}

void DepthImage::LocalCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& local_cloud)
{
    for(size_t i=0;i<cloud->points.size();i++){
        if(-threshold<=cloud->points[i].x && cloud->points[i].x<=threshold 
           && -threshold<=cloud->points[i].y && cloud->points[i].y<=threshold)
            local_cloud->points.push_back(cloud->points[i]);
    }
    cout<<"----Cloal Map Size:"<<local_cloud->points.size()<<endl;
}

void DepthImage::inverseCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                              pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& inverse_cloud,
                              tf::Transform transform)
{
    tf::Transform inverse_transform;
    inverse_transform = transform.inverse();

    double x = inverse_transform.getOrigin().x();
    double y = inverse_transform.getOrigin().y();
    double z = inverse_transform.getOrigin().z();
    tf::Quaternion q = inverse_transform.getRotation();
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll ,pitch, yaw);

    pcl_ros::transformPointCloud(*cloud, *inverse_cloud, inverse_transform);

    cout<<setprecision(3)<<"----Laser Frame --> Gloabl Frame" 
        <<" x:"<<x<<" y:"<<y<<" z:"<<z<<" roll:"<<roll<<" pitch:"<<pitch<<" yaw:"<<yaw<<endl;
}

void DepthImage::loadPCDFile(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    cout<<"Load :" <<FILE_PATH<<endl;
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (FILE_PATH, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("-----Couldn't read file\n");
    }
}

void DepthImage::main()
{
    loadPCDFile(map);
    cout<<"Start"<<endl;
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

