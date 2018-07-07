void SaveData::odomCallback(const OdometryConstPtr msg)
{
    odom = *msg;
    // transform_matrix = create_matrix(odom, 1.0);

	transform_listener();
    movement(msg);
    
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
    CloudAPtr threshold_cloud(new CloudA);
    
    pcl::fromROSMsg(*cloud, *input_cloud);
    for(size_t i=0;i<input_cloud->points.size();i++){
        double distance = sqrt(pow(input_cloud->points[i].x, 2)+ pow(input_cloud->points[i].y, 2) + pow(input_cloud->points[i].z, 2));
        if(distance < 30)
            threshold_cloud->points.push_back(input_cloud->points[i]);
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

// Odometryから移動量を算出
void SaveData::movement(const OdometryConstPtr odom)
{
    if(!odom_flag)
    {
        cout<<"first odom"<<endl;
        old_odom = *odom;
        odom_flag = true;
    }
    else{
        new_odom = *odom;
        double dt = sqrt( pow((new_odom.pose.pose.position.x - old_odom.pose.pose.position.x), 2) + 
                pow((new_odom.pose.pose.position.y - old_odom.pose.pose.position.y), 2) );
        distance += dt;
        old_odom = *odom;
    }
}

// global座標系のtfを取得
void SaveData::transform_listener()
{
	try{
		ros::Time now = ros::Time::now();
		global_listener.waitForTransform(global_frame, laser_frame, now, ros::Duration(0.02));
		global_listener.lookupTransform(global_frame, laser_frame,  now, global_transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(0.02).sleep();
	}
	// Publish Transform
    // geometry_msgs::Transform transform;
	// tf::transformTFToMsg(global_transform, transform);
	// transform_pub.publish(transform);
}

// savepointに到達したかを判定し, stop_flagによりsq2を停止
bool SaveData::check_savepoint()
{
    if(threshold < distance){
        // cout<<"arrive save point!!! Publish Stop Flag!!!"<<endl;
        stop_flag.data = true;
        stop_pub.publish(stop_flag);
        return true;
    }
    else{
        // cout<<"move to save point!!!"<<endl;
		// printf("distance:%.2f / %.2f\n", distance, threshold);
        stop_flag.data = false;
        stop_pub.publish(stop_flag);
        return false;
    }
}

// savepointに到達し, かつ停止が確認された場合
// optical_flowにより動的物体がカメラの画角内にいないか判定する
// 動的物体がない場合はsave_flag=trueとし, 点群を保存するプロセス save_pointcloudへ移行
// 確認された場合は一定回数(trial_count)試行
// 失敗した場合, reset()により次のsavepointを目指す
void SaveData::save_data()
{
	double vel = sqrt( pow(odom.twist.twist.linear.x, 2) + pow(odom.twist.twist.linear.y, 2) );
    if(arrival && vel < 0.01)
    {
        // cout<<"Stand by OK!!!!!"<<endl;

        if(!zed0_data && !zed1_data && !zed2_data){
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


// PointCloudを保存するメインプロセス
void SaveData::save_process()
{
	// Save Node Infomation
    sensor_fusion::Node node;
    node.header.frame_id = global_frame;
    node.node = node_num;
	tf::transformTFToMsg(global_transform, node.laser_transform);
	tf::transformTFToMsg(zed0_transform, node.zed0_transform);
	tf::transformTFToMsg(zed1_transform, node.zed1_transform);
	tf::transformTFToMsg(zed2_transform, node.zed2_transform);
	node.zed0_image = *zed0_image;
	node.zed0_cinfo = *zed0_cinfo;
	node.zed1_image = *zed1_image;
	node.zed1_cinfo = *zed1_cinfo;
	node.zed2_image = *zed2_image;
	node.zed2_cinfo = *zed2_cinfo;

    cout<<"save process"<<endl;
    ColorCloudAPtr zed0_cloud(new ColorCloudA);
    ColorCloudAPtr zed1_cloud(new ColorCloudA);
    ColorCloudAPtr zed2_cloud(new ColorCloudA);

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

    cout<<"Integrate PointCloud"<<endl;
    ColorCloudAPtr zed_cloud(new ColorCloudA);
    zed_cloud->header.frame_id = laser_frame;
    *zed_cloud += *zed0_cloud;
    *zed_cloud += *zed1_cloud;
    *zed_cloud += *zed2_cloud;

    // Normal Estimation
    // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    // normal_estimation(zed_cloud, normal_cloud);
   
    // Transform Pointcloud for global
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    global_pointcloud(zed_cloud, global_cloud);
    
    // Publish PointCloud
    pub_cloud(global_cloud, global_frame, global_pub);

    // Publish Node info
	cout<<"publish node"<<endl;
	ros::Time time = ros::Time::now();
    node.header.stamp = time;
	node.zed0_image.header.stamp = time;
	node.zed0_cinfo.header.stamp = time;
	node.zed1_image.header.stamp = time;
	node.zed1_cinfo.header.stamp = time;
	node.zed2_image.header.stamp = time;
	node.zed2_cinfo.header.stamp = time;
	
	node_pub.publish(node);
    // pcl::toROSMsg(*global_cloud, node.cloud);
    // node.cloud.header.frame_id = global_frame;
    // node.cloud.header.stamp = ros::Time::now();
	
    // SavePCDFile
    savePCDFile(global_cloud, node_num);
}

void SaveData::camera_process(CloudAPtr cloud, 
                    CameraInfoConstPtr cinfo, 
                    ImageConstPtr image,
                    tf::StampedTransform stamp_transform,
                    string target_frame, 
                    string source_frame,
                    ColorCloudAPtr &output_cloud)
{
    cout<<"Camera Process : "<<"target_frame:"<<target_frame<<" source_frame:"<<source_frame<<endl;
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

    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(q_x, q_y, q_z, q_w)).getRPY(roll ,pitch, yaw);
    printf("---x:%.2f y:%.2f z:%.2f roll:%.2f pitch:%.2f yaw:%.2f\n", x, y, z, roll, pitch, yaw);


    CloudAPtr trans_cloud(new CloudA);
    transform_pointcloud(cloud, trans_cloud, transform, target_frame, source_frame);
    
    ColorCloudAPtr pickup_cloud(new ColorCloudA);
    pickup_pointcloud(trans_cloud, pickup_cloud, image, cinfo);

    // CloudAPtr inverse_cloud(new CloudA);
    inverse_pointcloud(pickup_cloud, output_cloud, transform, target_frame, source_frame);
}



void SaveData::transform_pointcloud(CloudAPtr cloud,
                                    CloudAPtr& trans_cloud,
                                    tf::Transform transform,
                                    string target_frame, 
                                    string source_frame)
{
    pcl_ros::transformPointCloud(*cloud, *trans_cloud, transform);
    trans_cloud->header.frame_id = target_frame;
    cout<<"---TF Cloud"<<" Frame:"<<trans_cloud->header.frame_id<<" Size:"<<trans_cloud->points.size()<<endl;
}

void SaveData::pickup_pointcloud(CloudAPtr cloud, 
                       ColorCloudAPtr& pickup_cloud,
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
            ColorPointA p;
            p.x = (*pt).x;
            p.y = (*pt).y;
            p.z = (*pt).z;
            p.b = image.at<cv::Vec3b>(uv)[0];
            p.g = image.at<cv::Vec3b>(uv)[1];
            p.r = image.at<cv::Vec3b>(uv)[2];
            pickup_cloud->points.push_back(p);
        }
    }
    cout<<"---Pickup Cloud"<<" Frame:"<<pickup_cloud->header.frame_id<<" Size:"<<pickup_cloud->points.size()<<endl;
}

void SaveData::inverse_pointcloud(ColorCloudAPtr cloud,
                                  ColorCloudAPtr& inverse_cloud,
                                  tf::Transform transform,
                                  string target_frame,
                                  string source_frame)
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
    inverse_cloud->header.frame_id = source_frame;
    cout<<"---Inverse Cloud"<<" Frame:"<<inverse_cloud->header.frame_id<<" Size:"<<inverse_cloud->points.size()<<endl;

    printf("---x:%.2f y:%.2f z:%.2f roll:%.2f pitch:%.2f yaw:%.2f\n", x, y, z, roll, pitch, yaw);
}

void SaveData::normal_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                                 pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& normal_cloud)
{
	cout<<"Normal Estimation"<<endl;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.2);
    ne.compute(*normals);

    pcl::PointXYZRGBNormal tmp;
    for(size_t i=0;i<cloud->points.size();i++)
    {
		tmp.x = cloud->points[i].x;
		tmp.y = cloud->points[i].y;
		tmp.z = cloud->points[i].z;
		tmp.r = cloud->points[i].r;
		tmp.g = cloud->points[i].g;
		tmp.b = cloud->points[i].b;
		if(!isnan(normals->points[i].normal_x)){
			tmp.normal_x = normals->points[i].normal_x;
		}
		else{
			tmp.normal_x = 0.0;
		}
		if(!isnan(normals->points[i].normal_y)){
			tmp.normal_y = normals->points[i].normal_y;
		}
		else{
			tmp.normal_y = 0.0;
		}
		if(!isnan(normals->points[i].normal_z)){
			tmp.normal_z = normals->points[i].normal_z;
		}
		else{
			tmp.normal_z = 0.0;
		}
		if(!isnan(normals->points[i].curvature)){
			tmp.curvature = normals->points[i].curvature;
		}
		else{
			tmp.curvature = 0.0;
		}
		normal_cloud->points.push_back(tmp);
	}
}

void SaveData::global_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr& global_cloud)
{
    cout<<"Transform for Global"<<endl;
	tf::Transform transform;
    double x   = global_transform.getOrigin().x();
    double y   = global_transform.getOrigin().y();
    double z   = global_transform.getOrigin().z();
    double q_x = global_transform.getRotation().x();
    double q_y = global_transform.getRotation().y();
    double q_z = global_transform.getRotation().z();
    double q_w = global_transform.getRotation().w();
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(q_x, q_y, q_z, q_w));
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(q_x, q_y, q_z, q_w)).getRPY(roll ,pitch, yaw);
    printf("---x:%.2f y:%.2f z:%.2f roll:%.2f pitch:%.2f yaw:%.2f\n", x, y, z, roll, pitch, yaw);
    pcl_ros::transformPointCloud(*cloud, *global_cloud, transform);
}


void SaveData::pub_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                         string frame, 
                         ros::Publisher pub)
{
    cout<<"Publish PointCLoud"<<endl;
    PointCloud2 pc2;
    pcl::toROSMsg(*cloud, pc2);
    pc2.header.frame_id = frame;
    pc2.header.stamp = ros::Time::now();
    pub.publish(pc2);
}

void SaveData::savePCDFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                           int count)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*cloud, *save_cloud);
 
	save_cloud->width = 1;
	save_cloud->height = save_cloud->points.size();

    string file_name = to_string(count);
    // string path = HOME_DIRS + "/" + FILE_PATH + "/" + SAVE_PATH;
    // pcl::io::savePCDFileASCII("/home/amsl/PCD/Save/"+file_name+".pcd", *cloud);
	pcl::io::savePCDFile("/home/amsl/PCD/Save/"+file_name+".pcd", *save_cloud);
    printf("Num:%d saved %d\n", count, int(cloud->points.size()));
}
