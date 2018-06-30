void SaveData::odomCallback(const OdometryConstPtr msg)
{
    odom = *msg;
    transform_matrix = create_matrix(odom, 1.0);

    if(!odom_flag)
    {
        old_odom = *msg;
        odom_flag = false;
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
}

void SaveData::cloudCallback(const PointCloud2ConstPtr msg)
{
    if(save_flag)
        save_pointcloud(msg);
    else
        save_count = 0;
}

void SaveData::zed0_callback(const ImageConstPtr& image, const CameraInfoConstPtr& cinfo)
{
    printf("zed0 callback\n");
    zed0_image = image;
    zed0_cinfo = cinfo;

    try{
        ros::Time now = ros::Time::now();
        zed0_listener.waitForTransform("/centerlaser", "/zed0/zed_left_camera", now, ros::Duration(1.0));
        zed0_listener.lookupTransform("/centerlaser", "/zed0/zed_left_camera",  now, zed0_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void SaveData::zed1_callback(const ImageConstPtr& image, const CameraInfoConstPtr& cinfo)
{
    printf("zed1 callback\n");
    zed1_image = image;
    zed1_cinfo = cinfo;

    try{
        ros::Time now = ros::Time::now();
        zed1_listener.waitForTransform("/centerlaser", "/zed1/zed_left_camera", now, ros::Duration(1.0));
        zed1_listener.lookupTransform("/centerlaser", "/zed1/zed_left_camera",  now, zed1_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void SaveData::zed2_callback(const ImageConstPtr& image, const CameraInfoConstPtr& cinfo)
{
    cout<<"zed2 Callback"<<endl;
    zed2_image = image;
    zed2_cinfo = cinfo;

    try{
        ros::Time now = ros::Time::now();
        zed2_listener.waitForTransform("/centerlaser", "/zed2/zed_left_camera", now, ros::Duration(1.0));
        zed2_listener.lookupTransform("/centerlaser", "/zed2/zed_left_camera",  now, zed2_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void SaveData::zed0_optical_flow_calback(const BoolConstPtr msg)
{
    printf("zed0 optical_flow\n");
    zed0_data = msg->data;
}

void SaveData::zed1_optical_flow_calback(const BoolConstPtr msg)
{
    printf("zed1 optical_flow\n");
    zed1_data = msg->data;
}

void SaveData::zed2_optical_flow_calback(const BoolConstPtr msg)
{
    printf("zed2 optical_flow\n");
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
        *save_cloud += *threshold_cloud;
    count++;
}

bool SaveData::check_savepoint()
{
    if(threshold < distance){
        cout<<"arrive save point!!! Publish Stop Flag!!!"<<endl;
        emergency_flag.data = true;
        emergency_pub.publish(emergency_flag);
        return true;
    }
    else{
        cout<<"arrive save point!!! Publish Stop Flag!!!"<<endl;
        emergency_flag.data = false;
        emergency_pub.publish(emergency_flag);
        return false;
    }
}

void SaveData::save_data()
{
	double vel = sqrt( pow(odom.twist.twist.linear.x, 2) + pow(odom.twist.twist.linear.y, 2) );
    if(arrival && vel < 0.00001)
    {
        cout<<"Stanby OK!!!!!"<<endl;
        cout<<"check world movement..."<<endl;

        if(zed0_data && zed1_data && zed2_data)
            cout<<"world is stop"<<endl;
        else
            cout<<"zed0:"<<zed0_data<<" zed1:"<<zed1_data<<" zed2:"<<zed2_data<<endl;
    }
    else if(arrival)
    {
        cout<<"speed down and reset optical_flow "<<endl;
        Bool reset;
        reset.data = true;
        optical_flow_reset_pub.publish(reset);
    }
}

