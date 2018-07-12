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

    CloudAPtr local_cloud(new CloudA);
    CloudAPtr obstacle_cloud(new CloudA);
    CloudAPtr ground_cloud(new CloudA);
    CloudAPtr integrate_cloud(new CloudA);

    CloudAPtr pickup_cloud(new CloudA);
    CloudAPtr pickup_cloud_obstacle(new CloudA);
    CloudAPtr pickup_cloud_ground(new CloudA);

    CloudAPtr zed0_cloud(new CloudA);
    CloudAPtr zed0_cloud_obstacle(new CloudA);
    CloudAPtr zed0_cloud_ground(new CloudA);

    CloudAPtr zed1_cloud(new CloudA);
    CloudAPtr zed1_cloud_obstacle(new CloudA);
    CloudAPtr zed1_cloud_ground(new CloudA);

    CloudAPtr zed2_cloud(new CloudA);
    CloudAPtr zed2_cloud_obstacle(new CloudA);
    CloudAPtr zed2_cloud_ground(new CloudA);

    // 保存したMapの座標系はGlobal(Map)座標系になっている
    // laser_transform(global -- laser)の逆行列を使って laser座標系に変換する
    inverseCloud(obstacle_map, obstacle_cloud, laser_transform);
    inverseCloud(ground_map,   ground_cloud,   laser_transform);
    *local_cloud += *obstacle_cloud;
    *local_cloud += *ground_cloud;

    // Laser座標系を中心としてthreshold以内の点群をMapから取得
    LocalCloud(local_cloud, pickup_cloud);
    LocalCloud(obstacle_cloud, pickup_cloud_obstacle);
    LocalCloud(ground_cloud, pickup_cloud_ground);
    
    // laser座標系に変換したMapデータをカメラ座標に変換
    transform_pointcloud(pickup_cloud_obstacle, zed0_cloud_obstacle, zed0_transform, zed0_frame, laser_frame);
    transform_pointcloud(pickup_cloud_ground  , zed0_cloud_ground  , zed0_transform, zed0_frame, laser_frame);

    // transform_pointcloud(pickup_cloud, zed0_cloud, zed0_transform, zed0_frame, laser_frame);
    // transform_pointcloud(pickup_cloud, zed1_cloud, zed1_transform, zed1_frame, laser_frame);
    // transform_pointcloud(pickup_cloud, zed2_cloud, zed2_transform, zed2_frame, laser_frame);

    // DepthImageを作成
    cout<<"----zed0"<<endl;
    depthimage_creater(zed0_cloud_obstacle, zed0_cloud_ground,
                       zed0_image, zed0_cinfo, zed0_frame,
                       zed0_pub, zed0_raw_pub, zed0_cluster_pub, zed0_cloud_pub);
    
    // cout<<"----zed1"<<endl;
    // depthimage_creater(zed1_cloud, zed1_image, zed1_cinfo, zed1_frame, zed1_pub, zed1_raw_pub, zed1_cluster_pub, zed1_cloud_pub);
    
    // cout<<"----zed2"<<endl;
    // depthimage_creater(zed2_cloud, zed2_image, zed2_cinfo, zed2_frame, zed2_pub, zed2_raw_pub, zed2_cluster_pub, zed2_cloud_pub);

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

void DepthImage::depthimage_creater(CloudAPtr obstacle_cloud,
                                    CloudAPtr ground_cloud,
                                    sensor_msgs::ImageConstPtr image_msg,
                                    sensor_msgs::CameraInfoConstPtr cinfo_msg,
                                    string target_frame,
                                    image_transport::Publisher image_pub,
                                    ros::Publisher image_raw_pub,
                                    ros::Publisher cluster_pub,
                                    ros::Publisher cloud_pub)
{
    CloudAPtr integrate_cloud(new CloudA);
    *integrate_cloud += *obstacle_cloud;
    *integrate_cloud += *ground_cloud;

    // Visualize用
    sensor_msgs::Image tmp_image = *image_msg;
    
    // カメラの画角内の点群を参照点群として取得
    CloudAPtr reference_cloud(new CloudA);
    CloudAPtr reference_obstacle_cloud(new CloudA);
    CloudAPtr reference_ground_cloud(new CloudA);
    
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

    // カメラの画角内の点群を参照点として取得
    for(pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator pt=obstacle_cloud->points.begin(); pt<obstacle_cloud->points.end(); pt++)
    {
        if ((*pt).x<0) continue;
        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);

        if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows)
            reference_obstacle_cloud->points.push_back(*pt);
    }

    for(pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator pt=ground_cloud->points.begin(); pt<ground_cloud->points.end(); pt++)
    {
        if ((*pt).x<0) continue;
        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);

        if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows)
            reference_ground_cloud->points.push_back(*pt);
    }

    // 参照点群を可視化
    // *reference_cloud += *reference_obstacle_cloud;
    // *reference_cloud += *reference_ground_cloud;
    for(size_t i=0;i<reference_obstacle_cloud->points.size();i++){
        PointA p = reference_obstacle_cloud->points[i];
        p.r = 255;
        p.g = 0;
        p.b = 0;
        reference_cloud->points.push_back(p);
    }
    for(size_t i=0;i<reference_ground_cloud->points.size();i++){
        PointA p = reference_ground_cloud->points[i];
        p.r = 0;
        p.g = 0;
        p.b = 255;
        reference_cloud->points.push_back(p);
    }
    

    /*{{{
    double distance[cv_img_ptr->image.rows][cv_img_ptr->image.cols];
    memset(&distance, threshold, cv_img_ptr->image.rows*cv_img_ptr->image.cols);

    for(pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator pt=cloud->points.begin(); pt<cloud->points.end(); pt++)
    {
        if ((*pt).x<0) continue;
        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);

        if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows){
            double range = sqrt( pow((*pt).x, 2.0) + pow((*pt).y, 2.0) + pow((*pt).z, 2.0));
            if(range<threshold){
                for(int i=-1;i<=1;i++){
                    for(int j=-1;j<=1;j++){
                        int x = uv.x + i;
                        int y = uv.y + j;
                        if(0<x && x<image.cols && 0<y && y<image.rows)
                            distance[int(y)][int(x)] = range;
                    }
                }
            }
            reference_cloud->points.push_back(*pt);
        }
    }
    }}}*/

    // Clusteringを行う
    // vector<Clusters> cluster_array;
    // clustering(reference_obstacle_cloud, cluster_array);
    // CloudAPtr cluster_cloud(new CloudA);
    // for(size_t i=0;i<cluster_array.size();i++)
    //     *cluster_cloud += cluster_array[i].points;
    // CloudPublisher(cluster_cloud, target_frame, cluster_pub);

    // IOUを計算
    // CloudAPtr iou_cloud(new CloudA);
    // iou(cluster_array, cinfo_msg, iou_cloud);

/*{{{
#pragma omp parallel for
    for(int y=0; y<image.rows; y++){
        for(int x=0; x<image.cols; x++){
            double range = distance[y][x];
            if(range<threshold){
                COLOR c = GetColor(int(range/50*255.0), 0, 255);
                cv::Point2d uv;
                uv.x = x;
                uv.y = y;
                cv::circle(image, uv, 3, cv::Scalar(int(255*c.b),int(255*c.g),int(255*c.r)), -1);
            }
            else
            {
                image.at<cv::Vec3b>(y, x)[0] = 0;
                image.at<cv::Vec3b>(y, x)[1] = 0;
                image.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }
    }
}}}*/
 
    double distance[cv_img_ptr->image.rows][cv_img_ptr->image.cols];
    bool obstacle_init[cv_img_ptr->image.rows][cv_img_ptr->image.cols];
    bool ground_init[cv_img_ptr->image.rows][cv_img_ptr->image.cols];

    memset(&distance, 0, cv_img_ptr->image.rows*cv_img_ptr->image.cols);
    memset(&obstacle_init, false, cv_img_ptr->image.rows*cv_img_ptr->image.cols);
    memset(&ground_init, false, cv_img_ptr->image.rows*cv_img_ptr->image.cols);

#pragma omp parallel for
    for(pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator pt=reference_obstacle_cloud->points.begin(); pt<reference_obstacle_cloud->points.end(); pt++)
    {
        double range = sqrt( pow((*pt).x, 2.0) + pow((*pt).y, 2.0) + pow((*pt).z, 2.0));

        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);
        
        for(int i=-1;i<=1;i++){
            for(int j=-1;j<=1;j++){
                int x = uv.x + i;
                int y = uv.y + j;
                if(0<x && x<image.cols && 0<y && y<image.rows)
                {
                    if(!obstacle_init[y][x])
                    {
                        distance[y][x] = range; 
                        obstacle_init[y][x] = true;
                    }   
                    else{
                        if(range<distance[y][x])
                            distance[y][x] = range;
                    }
                }
            }
        }

        /*{{{
        int x = uv.x;
        int y = uv.y;

        if(!obstacle_init[y][x])
        {
            distance[y][x] = range; 
            obstacle_init[y][x] = true;
        }   
        else{
            if(range<distance[y][x])
                distance[y][x] = range;
        }
        }}}*/
    }

#pragma omp parallel for
    for(pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator pt=reference_ground_cloud->points.begin(); pt<reference_ground_cloud->points.end(); pt++)
    {
        double range = sqrt( pow((*pt).x, 2.0) + pow((*pt).y, 2.0) + pow((*pt).z, 2.0));

        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);

        for(int i=-1;i<=1;i++){
            for(int j=-1;j<=1;j++){
                int x = uv.x + i;
                int y = uv.y + j;
                if(0<x && x<image.cols && 0<y && y<image.rows)
                {
                    if(!obstacle_init[y][x])
                    {
                        if(!ground_init[y][x])
                        {
                            distance[y][x] = range;
                            ground_init[y][x] = true;
                        }
                        else if(range<distance[y][x])
                            distance[y][x] = range;
                    }
                }
            }
        }
        
        /*{{{
        int x = uv.x;
        int y = uv.y;

        if(!obstacle_init[y][x])
        {
            if(!ground_init[y][x])
            {
                distance[y][x] = range;
                ground_init[y][x] = true;
            }
            else if(range<distance[y][x])
                distance[y][x] = range;
        }
        }}}*/
    }


#pragma omp parallel for
    for(int y=0; y<image.rows; y++){
        for(int x=0; x<image.cols; x++){
            if(obstacle_init[y][x] || ground_init[y][x]){
                double range = distance[y][x];
                COLOR c = GetColor(int(range/50*255.0), 0, 255);
                image.at<cv::Vec3b>(y, x)[0] = 255*c.b;
                image.at<cv::Vec3b>(y, x)[1] = 255*c.g;
                image.at<cv::Vec3b>(y, x)[2] = 255*c.r;
            }
            else{
                image.at<cv::Vec3b>(y, x)[0] = 0;
                image.at<cv::Vec3b>(y, x)[1] = 0;
                image.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }
    }

    // Publish Cloud
    CloudPublisher(reference_cloud, target_frame, cloud_pub);
    // Publish DepthImage 
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    image_pub.publish(msg);
    // Publish Raw Image
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

// PointCloudの情報をClusterに格納
void DepthImage::getClusterInfo(CloudA pt,
                                Cluster& cluster)
{
    Vector3f centroid;
    centroid[0]=pt.points[0].x;
    centroid[1]=pt.points[0].y;
    centroid[2]=pt.points[0].z;

    Vector3f min_p;
    min_p[0]=pt.points[0].x;
    min_p[1]=pt.points[0].y;
    min_p[2]=pt.points[0].z;

    Vector3f max_p;
    max_p[0]=pt.points[0].x;
    max_p[1]=pt.points[0].y;
    max_p[2]=pt.points[0].z;

    for(size_t i=1;i<pt.points.size();i++){
        centroid[0]+=pt.points[i].x;
        centroid[1]+=pt.points[i].y;
        centroid[2]+=pt.points[i].z;
        if (pt.points[i].x<min_p[0]) min_p[0]=pt.points[i].x;
        if (pt.points[i].y<min_p[1]) min_p[1]=pt.points[i].y;
        if (pt.points[i].z<min_p[2]) min_p[2]=pt.points[i].z;

        if (pt.points[i].x>max_p[0]) max_p[0]=pt.points[i].x;
        if (pt.points[i].y>max_p[1]) max_p[1]=pt.points[i].y;
        if (pt.points[i].z>max_p[2]) max_p[2]=pt.points[i].z;
    }

    cluster.x=centroid[0]/(float)pt.points.size();
    cluster.y=centroid[1]/(float)pt.points.size();
    cluster.z=centroid[2]/(float)pt.points.size();
    cluster.depth  = max_p[0]-min_p[0];
    cluster.width  = max_p[1]-min_p[1];
    cluster.height = max_p[2]-min_p[2]; 
    cluster.min_p = min_p;
    cluster.max_p = max_p;
}

// Clustering
void DepthImage::clustering(CloudAPtr cloud_in,
                            vector<Clusters>& cluster_array){
    //Downsample//
    pcl::VoxelGrid<PointA> vg;  
    CloudAPtr ds_cloud (new CloudA);  
    vg.setInputCloud (cloud_in);  
    vg.setLeafSize (0.1f, 0.1f, 0.1f);
    vg.filter (*ds_cloud);
    cout<<"----DownSampling:"<<ds_cloud->points.size()<<endl;

    //downsampled point's z =>0
    vector<float> tmp_z;
    tmp_z.resize(ds_cloud->points.size());
	for(int i=0;i<(int)ds_cloud->points.size();i++){
        tmp_z[i]=ds_cloud->points[i].z;
		ds_cloud->points[i].z  = 0.0;
    }
    //Clustering//
    pcl::search::KdTree<PointA>::Ptr tree (new pcl::search::KdTree<PointA>);
    tree->setInputCloud (ds_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointA> ec;
    ec.setClusterTolerance (0.15); // 15cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (1000000);
    ec.setSearchMethod (tree);
    ec.setInputCloud(ds_cloud);
    ec.extract (cluster_indices);
    //reset z value
	for(int i=0;i<(int)ds_cloud->points.size();i++)
        ds_cloud->points[i].z=tmp_z[i];

    for(int iii=0;iii<(int)cluster_indices.size();iii++)
    {
        // cluster points
        CloudAPtr cloud_cluster (new CloudA);
        cloud_cluster->points.resize(cluster_indices[iii].indices.size());
        // cluster data
        Cluster data;
        for(int jjj=0;jjj<int(cluster_indices[iii].indices.size());jjj++){
            int p_num = cluster_indices[iii].indices[jjj];
            cloud_cluster->points[jjj] = ds_cloud->points[p_num];
        }
        getClusterInfo(*cloud_cluster, data);

		PointA center;
		center.x = data.x;
		center.y = data.y;
		center.z = data.z;

        Clusters cluster;
        cluster.data = data;
        cluster.centroid.points.push_back(center);
        for(size_t i=0;i<cloud_cluster->points.size();i++)
            cluster.points.points.push_back(cloud_cluster->points[i]);

        cluster_array.push_back(cluster);
    }
    cout<<"----Clustering:"<<cluster_array.size()<<endl;
}

// 各クラスタのIOUを計算
void DepthImage::iou(vector<Clusters> cluster_array,
                     sensor_msgs::CameraInfoConstPtr cinfo_msg,
                     CloudAPtr& cloud)
{

    vector<Clusters> clusters = cluster_array;
    int id[int(clusters.size())]={};
    double distance[int(clusters.size())] = {};

    // 各クラスタの重心点の距離をidとdistanceに保存
    for(int i=0;i<int(clusters.size());i++)
    {
        id[i] = i;
        distance[i] = sqrt(pow(clusters[i].data.x, 2)+pow(clusters[i].data.y, 2));
    }

    // クラスタを距離が近い順に並び替え
    for(int i=0;i<int(clusters.size());i++){
        for(int j=i+1;j<int(clusters.size());j++){
            if(distance[i] > distance[j]){
                int tmp = id[i];
                id[i] = id[j];
                id[j] = tmp;

                double tmp_dis = distance[i];
                distance[i] = distance[j];
                distance[j] = tmp_dis;

                Clusters tmp_cls = clusters[i];
                clusters[i] = clusters[j];
                clusters[j] = tmp_cls;
            }
        }
    }

    for(int i=0;i<int(clusters.size());i++)
        printf("    ID:%2d x:%.2f y:%.2f z:%.2f W:%.2f H:%.2f D:%.2f Distance:%.2f min:%.2f %.2f %.2f max:%.2f %.2f %.2f\n", 
               i, clusters[i].data.x, clusters[i].data.y, clusters[i].data.z, 
               clusters[i].data.width,    clusters[i].data.height,   clusters[i].data.depth, distance[i], 
               clusters[i].data.min_p[0], clusters[i].data.min_p[1], clusters[i].data.min_p[2],
               clusters[i].data.max_p[0], clusters[i].data.max_p[1], clusters[i].data.max_p[2]);

    double iou[int(clusters.size())][int(clusters.size())] = {};
    memset(&iou, 0, clusters.size()*clusters.size());

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cinfo_msg);

    for(int i=0;i<int(clusters.size());i++){
        cv::Point3d cv_min(-clusters[i].data.min_p[1], -clusters[i].data.min_p[2], clusters[i].data.min_p[0]);
        cv::Point3d cv_max(-clusters[i].data.max_p[1], -clusters[i].data.max_p[2], clusters[i].data.max_p[0]);

        cv::Point2d uv_min = cam_model.project3dToPixel(cv_min);
        cv::Point2d uv_max = cam_model.project3dToPixel(cv_max);

        printf("%.2f %.2f %.2f %.2f \n", uv_min.x, uv_min.y, uv_max.x, uv_max.y);
    }


    for(int i=0;i<int(clusters.size());i++){
        for(int j=0;j<int(clusters.size());j++){
            cv::Point3d cv_i_min(-clusters[i].data.min_p[1], -clusters[i].data.min_p[2], clusters[i].data.min_p[0]);
            cv::Point3d cv_i_max(-clusters[i].data.max_p[1], -clusters[i].data.max_p[2], clusters[i].data.max_p[0]);
            cv::Point3d cv_j_min(-clusters[j].data.min_p[1], -clusters[j].data.min_p[2], clusters[j].data.min_p[0]);
            cv::Point3d cv_j_max(-clusters[j].data.max_p[1], -clusters[j].data.max_p[2], clusters[j].data.max_p[0]);

            cv::Point2d uv_i_min = cam_model.project3dToPixel(cv_i_min);
            cv::Point2d uv_i_max = cam_model.project3dToPixel(cv_i_max);
            cv::Point2d uv_j_min = cam_model.project3dToPixel(cv_j_min);
            cv::Point2d uv_j_max = cam_model.project3dToPixel(cv_j_max);

            ROI ROI_I, ROI_J;

            ROI_I.x_offset = uv_i_min.x;
            ROI_I.y_offset = uv_i_min.y;
            ROI_I.width    = uv_i_max.x - uv_i_min.x;
            ROI_I.height   = uv_i_max.y - uv_i_max.y;

            ROI_J.x_offset = uv_j_min.x;
            ROI_J.y_offset = uv_j_min.y;
            ROI_J.width    = uv_j_max.x - uv_j_min.x;
            ROI_J.height   = uv_j_max.y - uv_j_max.y;

            // printf("I:%d %.2f %.2f %.2f %.2f\n", i, ROI_I.x_offset, ROI_I.y_offset, ROI_I.width, ROI_I.height);
            // printf("J:%d %.2f %.2f %.2f %.2f\n", j, ROI_J.x_offset, ROI_J.y_offset, ROI_J.width, ROI_J.height);

            cv::Rect RECT_I(ROI_I.x_offset, ROI_I.y_offset, ROI_I.width, ROI_I.height);
            cv::Rect RECT_J(ROI_J.x_offset, ROI_J.y_offset, ROI_J.width, ROI_J.height);

            cv::Rect RECT_OVERLAP = RECT_I & RECT_J;

            int AREA_I          = RECT_I.width * RECT_I.height;
            int AREA_J          = RECT_J.width * RECT_J.height;
            int AREA_OVERLAP    = RECT_OVERLAP.width * RECT_OVERLAP.height;
            
            if(0.0<AREA_OVERLAP){
                int AREA_UNION = AREA_I + AREA_J - AREA_OVERLAP;
                iou[i][j] = AREA_OVERLAP / AREA_UNION;
            }
        }
    }
    for(int i=0;i<int(clusters.size());i++){
        for(int j=0;j<int(clusters.size());j++)
            printf("%.2f ", iou[i][j]);
        printf("\n");
    }


}


void DepthImage::main()
{
    loadPCDFile(obstacle_map, OBSTACLE_PATH);
    loadPCDFile(ground_map,   GROUND_PATH);
    cout<<"Start"<<endl;
}

void DepthImage::loadPCDFile(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, string file_path)
{
    cout<<"Load :" <<file_path<<endl;
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (file_path, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("-----Couldn't read file\n");
    }
}

void DepthImage::CloudPublisher(CloudAPtr cloud,
                                string target_frame,
                                ros::Publisher pub)
{
     sensor_msgs::PointCloud2 pc2;
     pcl::toROSMsg(*cloud, pc2);
     pc2.header.frame_id = target_frame;
     pc2.header.stamp = ros::Time::now();
     pub.publish(pc2);
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

