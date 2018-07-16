#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <sensor_fusion/Node.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>

typedef pcl::PointXYZRGBNormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;
using namespace Eigen;

class Reference{
    private:
        ros::NodeHandle nh;
        
        ros::Subscriber node_sub;

        ros::Publisher zed0_cloud_pub;
        ros::Publisher zed1_cloud_pub;
        ros::Publisher zed2_cloud_pub;

        ros::Publisher local_cloud_pub;

        //  Frame
        string global_frame;
        string laser_frame;
        string zed0_frame;
        string zed1_frame;
        string zed2_frame;
        
        // MAP File Path
        string OBSTACLE_PATH;
        string GROUND_PATH;
        CloudAPtr obstacle_map;
        CloudAPtr ground_map;

        // local cloud area
        int threshold;

    public:
        Reference();

        void nodeCallback(const sensor_fusion::NodeConstPtr msg);
        void transform_pointcloud(CloudAPtr cloud,
                                  CloudAPtr& trans_cloud,  
                                  tf::Transform transform,
                                  string target_frame,
                                  string source_frame);
        void LocalCloud(CloudAPtr cloud, CloudAPtr& local_cloud);
        
        void inverseCloud(CloudAPtr cloud, CloudAPtr& inverse_cloud, tf::Transform transform);

        void referenceCloud(CloudAPtr obstacle_cloud,
                            CloudAPtr ground_cloud,
                            CloudAPtr& reference_cloud,
                            sensor_msgs::ImageConstPtr image_msg,
                            sensor_msgs::CameraInfoConstPtr cinfo_msg,
                            string target_frame);

        void main();
        
        void loadPCDFile(CloudAPtr cloud, string file_path);
        
        void CloudPublisher (CloudAPtr cloud, string target_frame, ros::Publisher pub);

};

Reference::Reference()
    : nh("~"), 
      obstacle_map(new CloudA),
      ground_map(new CloudA)
{
    nh.getParam("global_frame", global_frame);
    nh.getParam("laser_frame" , laser_frame);
    nh.getParam("zed0_frame"  , zed0_frame);
    nh.getParam("zed1_frame"  , zed1_frame);
    nh.getParam("zed2_frame"  , zed2_frame);

    nh.getParam("obstacle_path", OBSTACLE_PATH);
    nh.getParam("ground_path",GROUND_PATH);

    nh.getParam("threshold", threshold);

    node_sub = nh.subscribe("/node", 10, &Reference::nodeCallback, this);

    zed0_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/zed0_reference_cloud/viewer", 10);
    zed1_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/zed1_reference_cloud/viewer", 10);
    zed2_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/zed2_reference_cloud/viewer", 10);

    local_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_map/viewer", 10);
}

void Reference::nodeCallback(const sensor_fusion::NodeConstPtr msg)
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

    CloudAPtr zed0_reference_cloud (new CloudA);
    CloudAPtr zed0_cloud_obstacle(new CloudA);
    CloudAPtr zed0_cloud_ground(new CloudA);

    CloudAPtr zed1_reference_cloud (new CloudA);
    CloudAPtr zed1_cloud_obstacle(new CloudA);
    CloudAPtr zed1_cloud_ground(new CloudA);

    CloudAPtr zed2_reference_cloud (new CloudA);
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

    // 参照点群を取得
    referenceCloud(zed0_cloud_obstacle, 
                   zed0_cloud_ground,
                   zed0_reference_cloud,
                   zed0_image, 
                   zed0_cinfo, 
                   zed0_frame);

    // Publish Cloud
    CloudPublisher(zed0_reference_cloud, zed0_frame, zed0_cloud_pub);
    CloudPublisher(local_cloud, laser_frame, local_cloud_pub);


}

void Reference::referenceCloud(CloudAPtr obstacle_cloud,
                               CloudAPtr ground_cloud,
                               CloudAPtr& reference_cloud,
                               sensor_msgs::ImageConstPtr image_msg,
                               sensor_msgs::CameraInfoConstPtr cinfo_msg,
                               string target_frame)
{
    CloudAPtr integrate_cloud(new CloudA);
    *integrate_cloud += *obstacle_cloud;
    *integrate_cloud += *ground_cloud;

    // カメラの画角内の点群を参照点群として取得
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
    *reference_cloud += *reference_obstacle_cloud;
    *reference_cloud += *reference_ground_cloud;
}

void Reference::LocalCloud(CloudAPtr cloud, CloudAPtr& local_cloud)
{
    for(size_t i=0;i<cloud->points.size();i++){
        if(-threshold<=cloud->points[i].x && cloud->points[i].x<=threshold 
           && -threshold<=cloud->points[i].y && cloud->points[i].y<=threshold)
            local_cloud->points.push_back(cloud->points[i]);
    }
    cout<<"----Cloal Map Size:"<<local_cloud->points.size()<<endl;
}

void Reference::transform_pointcloud(CloudAPtr cloud,
                                     CloudAPtr& trans_cloud,
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


void Reference::inverseCloud(CloudAPtr cloud, CloudAPtr& inverse_cloud, tf::Transform transform)
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


void Reference::main()
{
    loadPCDFile(obstacle_map, OBSTACLE_PATH);
    loadPCDFile(ground_map,   GROUND_PATH);
    cout<<"Start"<<endl;
}

void Reference::loadPCDFile(CloudAPtr cloud, string file_path)
{
    cout<<"Load :" <<file_path<<endl;
    
    if (pcl::io::loadPCDFile<PointA> (file_path, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("-----Couldn't read file\n");
    }
}

void Reference::CloudPublisher(CloudAPtr cloud,
                                string target_frame,
                                ros::Publisher pub)
{
     sensor_msgs::PointCloud2 pc2;
     pcl::toROSMsg(*cloud, pc2);
     pc2.header.frame_id = target_frame;
     pc2.header.stamp = ros::Time::now();
     pub.publish(pc2);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reference_viewer");

    Reference re;

    re.main();

    ros::spin();

    return 0;
}
