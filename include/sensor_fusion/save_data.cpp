void SaveData::odomCallback(const OdometryConstPtr msg)
{
    odom = *msg;
    transform_matrix = create_matrix(odom, 1.0);

    if(!odom_flag)
    {
		cout<<"first odom"<<endl;
        old_odom = *msg;
        odom_flag = true;
    }
    else{
        new_odom = *msg;
        // 移動量を算出
        double dt = sqrt( pow((new_odom.pose.pose.position.x - old_odom.pose.pose.position.x), 2) + 
                pow((new_odom.pose.pose.position.y - old_odom.pose.pose.position.y), 2) );
        distance += dt;
        old_odom = *msg;
    }

    arrival = check_savepoint();

    save_data();
}

void SaveData::cloudCallback(const PointCloud2ConstPtr msg)
{
    if(save_flag)
        save_pointcloud(msg);
    else
        count = 0;
}

void SaveData::zed0_callback(const ImageConstPtr& image, const CameraInfoConstPtr& cinfo)
{
    // printf("zed0 callback\n");
    zed0_image = image;
    zed0_cinfo = cinfo;

    try{
        ros::Time now = ros::Time::now();
        zed0_listener.waitForTransform(zed0_frame, laser_frame, now, ros::Duration(1.0));
        zed0_listener.lookupTransform(zed0_frame, laser_frame,  now, zed0_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void SaveData::zed1_callback(const ImageConstPtr& image, const CameraInfoConstPtr& cinfo)
{
    // printf("zed1 callback\n");
    zed1_image = image;
    zed1_cinfo = cinfo;

    try{
        ros::Time now = ros::Time::now();
        zed1_listener.waitForTransform(zed1_frame, laser_frame, now, ros::Duration(1.0));
        zed1_listener.lookupTransform(zed1_frame, laser_frame,  now, zed1_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void SaveData::zed2_callback(const ImageConstPtr& image, const CameraInfoConstPtr& cinfo)
{
    // cout<<"zed2 Callback"<<endl;
    zed2_image = image;
    zed2_cinfo = cinfo;

    try{
        ros::Time now = ros::Time::now();
        zed2_listener.waitForTransform(zed2_frame, laser_frame, now, ros::Duration(1.0));
        zed2_listener.lookupTransform(zed2_frame, laser_frame,  now, zed2_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void SaveData::zed0_optical_flow_calback(const BoolConstPtr msg)
{
    // printf("zed0 optical_flow\n");
    zed0_data = msg->data;
}

void SaveData::zed1_optical_flow_calback(const BoolConstPtr msg)
{
    // printf("zed1 optical_flow\n");
    zed1_data = msg->data;
}

void SaveData::zed2_optical_flow_calback(const BoolConstPtr msg)
{
    // printf("zed2 optical_flow\n");
    zed2_data = msg->data;
}

void SaveData::save_pointcloud(const PointCloud2ConstPtr cloud)
{
    CloudAPtr input_cloud(new CloudA);
    CloudAPtr transform_cloud(new CloudA);
    CloudAPtr threshold_cloud(new CloudA);
    
    pcl::fromROSMsg(*cloud, *input_cloud);
	pcl::transformPointCloud(*input_cloud, *transform_cloud, transform_matrix);
	
    for(size_t i=0;i<input_cloud->points.size();i++){
        double distance = sqrt(pow(input_cloud->points[i].x, 2)+ pow(input_cloud->points[i].y, 2) + pow(input_cloud->points[i].z, 2));
        if(distance < 30)
            threshold_cloud->points.push_back(transform_cloud->points[i]);
    }

    if(count < save_count)
	{
		*save_cloud += *threshold_cloud;
		if(count % 100 == 0) printf("count:%d/%d Cloud_Size:%d\n", count, save_count, int(save_cloud->points.size()));
	}

	if(count == save_count)
	{
		cout<<"Node:"<<node_num<<" Success Save PointCloud!!! Next Node"<<endl;
        save_process();
		reset();
	}
    count++;
}

bool SaveData::check_savepoint()
{
    if(threshold < distance){
        // cout<<"arrive save point!!! Publish Stop Flag!!!"<<endl;
        emergency_flag.data = true;
        emergency_pub.publish(emergency_flag);
        return true;
    }
    else{
        // cout<<"move to save point!!!"<<endl;
		printf("distance:%.2f / %.2f\n", distance, threshold);
        emergency_flag.data = false;
        emergency_pub.publish(emergency_flag);
        return false;
    }
}

void SaveData::save_data()
{
	double vel = sqrt( pow(odom.twist.twist.linear.x, 2) + pow(odom.twist.twist.linear.y, 2) );
    if(arrival && vel < 0.01)
    {
        // cout<<"Stand by OK!!!!!"<<endl;

        if(zed0_data && zed1_data && zed2_data){
			// cout<<"   world is stop"<<endl;
			save_flag = true;
		}
        else{
			count = 0;
			save_cloud->points.clear();
			fail_count++;
            cout<<"   Fail to Save PointCloud!!!!! "<<"FAIL COUNT:"<<fail_count<<endl;;
		}

		if(trial_count == fail_count)
		{
			cout<<"Node:"<<node_num<<" Fail... Move to Next Node"<<endl;
			reset();
		}
    }
    else if(arrival)
    {
        cout<<"speed down"<<endl;
    }
}

void SaveData::reset()
{
	node_num++;
	fail_count = 0;
	count = 0;
	distance = 0;
	save_cloud->points.clear();
	arrival = false;
	save_flag = false;
}

void SaveData::save_process()
{
    cout<<"save process"<<endl;
    CloudAPtr zed0_cloud(new CloudA);
    CloudAPtr zed1_cloud(new CloudA);
    CloudAPtr zed2_cloud(new CloudA);

    // zed0
    camera_process(save_cloud, zed0_cinfo, zed0_image,
                   zed0_transform,
                   zed0_frame, laser_frame,
                   zed0_cloud);
    // zed1 
    camera_process(save_cloud, zed1_cinfo, zed1_image,
            zed1_transform,
            zed1_frame, laser_frame,
            zed1_cloud);
    // zed2                                                                 
    camera_process(save_cloud, zed2_cinfo, zed2_image,
            zed2_transform,
            zed2_frame, laser_frame,
            zed2_cloud);

}

void SaveData::camera_process(CloudAPtr cloud, 
                    CameraInfoConstPtr cinfo, 
                    ImageConstPtr image,
                    tf::StampedTransform stamp_transform,
                    string target_frame, 
                    string source_frame,
                    CloudAPtr &output)
{
    cout<<"Camera Process : "<<"target_frame"<<endl;
    tf::Transform transform;
    double x = stamp_transform.getOrigin().x();
    double y = stamp_transform.getOrigin().y();
    double z = stamp_transform.getOrigin().z();
    double q_x = stamp_transform.getRotation().x();
    double q_y = stamp_transform.getRotation().y();
    double q_z = stamp_transform.getRotation().z();
    double q_w = stamp_transform.getRotation().w();
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(q_x, q_y, q_z, q_w));

    CloudAPtr trans_cloud(new CloudA);
    transform_pointcloud(cloud, trans_cloud, transform, target_frame, source_frame);
    
    CloudAPtr pickup_cloud(new CloudA);
    pickup_pointcloud(trans_cloud, pickup_cloud, image, cinfo);
}



void SaveData::transform_pointcloud(CloudAPtr cloud,
                                    CloudAPtr& trans_cloud,
                                    tf::Transform transform,
                                    string target_frame, 
                                    string source_frame)
{
    // cout<<"TF PointCloud..."<<"target_frame"<<"---->"<<source_frame<<endl;
    pcl_ros::transformPointCloud(*cloud, *trans_cloud, transform);
    cout<<"---TF Cloud"<<" Frame:"<<trans_cloud->header.frame_id<<" Size:"<<trans_cloud->points.size()<<endl;
}

void SaveData::pickup_pointcloud(CloudAPtr cloud, 
                       CloudAPtr& pickup_cloud,
                       ImageConstPtr image_msg,
                       CameraInfoConstPtr cinfo_msg)
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

    pickup_cloud->header.frame_id = cloud->header.frame_id;

    for(CloudA::iterator pt = cloud->points.begin(); pt<cloud->points.end(); pt++)
    {
        if ((*pt).x<0) continue;
        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);

        if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows){
            PointA p;
            p.x = (*pt).x;
            p.y = (*pt).y;
            p.z = (*pt).z;
            // p.b = image.at<cv::Vec3b>(uv)[0];
            // p.g = image.at<cv::Vec3b>(uv)[1];
            // p.r = image.at<cv::Vec3b>(uv)[2];
            pickup_cloud->points.push_back(p);
        }
    }
    cout<<"---pickup cloud..."<<pickup_cloud->points.size()<<endl;
}
