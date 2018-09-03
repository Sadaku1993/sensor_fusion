#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_fusion/NodeInfo.h>

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;
using namespace Eigen;

typedef struct{
    double r, g, b;
} COLOR;

class DepthImage{
    private:
        ros::NodeHandle nh;
        ros::Subscriber node_sub;
        // frame
        string global_frame;
        string laser_frame;
        string camera_frame;
        // transform (globel_frame:laser, child_frame:odom)
        tf::TransformListener laser_listener;
        tf::StampedTransform  laser_transform;
        // transform (global_frame:camera, child_frame:odom)
        tf::TransformListener camera_listener;
        tf::StampedTransform  camera_transform;
        // cloud
        CloudAPtr map_cloud;
        // node num
        int min_node;
        int max_node;
        // load path
        string load_path;
        string load_name;
        // save path
        string save_path;
        string save_name;

    public:
        DepthImage();

        void nodeCallback(const sensor_fusion::NodeInfoConstPtr msg);

        // transform pointcloud
        void transform_pointcloud(CloudAPtr cloud,
                                  CloudAPtr &trans_cloud,
                                  tf::StampedTransform transform,
                                  string target_frame,
                                  string source_frame);
        // quiclsort
        double distance(PointA pt);
        void pcl_quicksort(CloudA& cloud, int left, int right);

        // depthimage
        COLOR GetColor(double v, double vmin, double vmax);
        
        void start();

        // save or load process
        void savePCDFile(CloudAPtr cloud, string file_path, string file_name);
        void loadPCDFile(CloudAPtr, string file_path, string file_name);
        void saveImageFile(sensor_msgs::ImageConstPtr image, string file_path, string file_name);

};

DepthImage::DepthImage()
    : nh("~"), map_cloud(new CloudA)
{
    nh.getParam("global_frame", global_frame);
    nh.getParam("laser_frame" , laser_frame);
    nh.getParam("camera_frame", camera_frame);

    nh.param<int>("min_node", min_node, 0);
    nh.param<int>("max_node", max_node, 0);

    nh.param<string>("load_path", load_path, "/home/amsl/SII/Map/");
    nh.param<string>("load_name", load_name, "map.pcd");
    nh.param<string>("save_path", save_path, "/home/amsl/SII/Depth/");

    // callback
    node_sub = nh.subscribe("/node", 10, &DepthImage::nodeCallback, this);
}

void DepthImage::nodeCallback(const sensor_fusion::NodeInfoConstPtr msg)
{
    printf("Node:%d\n", int(msg->node));

    // transform listener (global_frame:camera child_frame:odom)
    try{
        ros::Time time = ros::Time::now();
        camera_listener.waitForTransform(camera_frame, global_frame, time, ros::Duration(1.0));
        camera_listener.lookupTransform(camera_frame, global_frame, time, camera_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
    }

    // transform pointcloud (header_frame:camera)
    CloudAPtr trans_cloud(new CloudA);
    transform_pointcloud(map_cloud, trans_cloud, camera_transform, camera_frame, global_frame);

    // callback image and camerainfo data
    sensor_msgs::Image::Ptr image_msg(new sensor_msgs::Image);
    sensor_msgs::CameraInfo::Ptr cinfo_msg(new sensor_msgs::CameraInfo);
    *image_msg = msg->image;
    *cinfo_msg = msg->cinfo;
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

    // pickup pointcloud(camera area)
    CloudAPtr reference_cloud(new CloudA);
    for(CloudA::iterator pt=trans_cloud->points.begin(); pt<trans_cloud->points.end(); pt++)
    {
        if ((*pt).x<0) continue;
        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);
        if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows)
            reference_cloud->points.push_back(*pt);
    }
    std::cout<<"----Image width:"<<image.cols<<" height:"<<image.rows<<std::endl;
    std::cout<<"----Cloud Size:"<<reference_cloud->points.size()<<std::endl;

    // quicksort
    // 点群をカメラ座標系からの距離が短い順に昇順ソート
    pcl_quicksort(*reference_cloud, 0, int(reference_cloud->points.size()-1));

    // depthImage
    double distance[cv_img_ptr->image.rows][cv_img_ptr->image.cols];
    bool init[cv_img_ptr->image.rows][cv_img_ptr->image.cols];
    memset(&distance, 0, cv_img_ptr->image.rows*cv_img_ptr->image.cols);
    memset(&init, false, cv_img_ptr->image.rows*cv_img_ptr->image.cols);


#pragma omp parallel for
    for(CloudA::iterator pt=reference_cloud->points.begin(); pt<reference_cloud->points.end(); pt++)
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
                    if(!init[y][x])
                    {
                        distance[y][x] = range; 
                        init[y][x] = true;
                    }   
                    else{
                        if(range<distance[y][x])
                            distance[y][x] = range;
                    }
                }
            }
        }
    }
#pragma omp parallel for
    for(int y=0; y<image.rows; y++){
        for(int x=0; x<image.cols; x++){
            if(init[y][x]){
                double range = distance[y][x];
                COLOR c = GetColor(int(range/50*255.0), 0, 255);
                image.at<cv::Vec3b>(y, x)[0] = 255*c.b;
                image.at<cv::Vec3b>(y, x)[1] = 255*c.g;
                image.at<cv::Vec3b>(y, x)[2] = 255*c.r;
            }
            // else{
            //     image.at<cv::Vec3b>(y, x)[0] = 0;
            //     image.at<cv::Vec3b>(y, x)[1] = 0;
            //     image.at<cv::Vec3b>(y, x)[2] = 0;
            // }
        }
    }

    string file_name = to_string(msg->node);
    cv::imwrite(save_path+file_name+".jpg", image);
    printf("Save Image File (cols:%d rows:%d)\n", image.cols, image.rows);
}

double DepthImage::distance(PointA point)
{
    double dis = sqrt( pow(point.x, 2.0) + pow(point.y, 2.0) + pow(point.z, 2.0));
    return dis;
}

void DepthImage::pcl_quicksort(CloudA &cloud, int left, int right)
{
    int l_hold, r_hold;
    double pivot;
    PointA tmp;

    l_hold = left;
    r_hold = right;
    pivot = distance(cloud.points[left]);

    tmp = cloud.points[left];

    while (left < right)
    {
        while ((distance(cloud.points[right]) >= pivot) && (left < right))
            right--;
        if (left != right)
        {
            cloud.points[left] = cloud.points[right];
            left++;
        }
        while ((distance(cloud.points[left]) <= pivot) && (left < right))
            left++;
        if (left != right)
        {
            cloud.points[right] = cloud.points[left];
            right--;
        }
    }
    cloud.points[left] = tmp;
    
    if(l_hold<left)
        pcl_quicksort(cloud, l_hold, left-1);
    if(r_hold > left)
        pcl_quicksort(cloud, left+1, r_hold);
}

// start
void DepthImage::start()
{
    printf("start!!!!\n");
    loadPCDFile(map_cloud, load_path, load_name);
}

// transform_pointclud
void DepthImage::transform_pointcloud(CloudAPtr cloud,
                                      CloudAPtr &trans_cloud,
                                      tf::StampedTransform stamped_transform,
                                      string target_frame,
                                      string source_frame)
{
    tf::Transform transform;
    double x = stamped_transform.getOrigin().x();
    double y = stamped_transform.getOrigin().y();
    double z = stamped_transform.getOrigin().z();
    double q_x = stamped_transform.getRotation().x();
    double q_y = stamped_transform.getRotation().y();
    double q_z = stamped_transform.getRotation().z();
    double q_w = stamped_transform.getRotation().w();
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(q_x, q_y, q_z, q_w));

    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(q_x, q_y, q_z, q_w)).getRPY(roll ,pitch, yaw);

    pcl_ros::transformPointCloud(*cloud, *trans_cloud, transform);
    trans_cloud->header.frame_id = target_frame;
}


// savePCDFile
void DepthImage::savePCDFile(CloudAPtr cloud, 
                             string file_path, 
                             string file_name)
{
    CloudAPtr copy_cloud(new CloudA);
    pcl::copyPointCloud(*cloud, *copy_cloud);
    copy_cloud->width = 1;
    copy_cloud->height = copy_cloud->points.size();
    pcl::io::savePCDFile(file_path+file_name+".pcd", *copy_cloud);
    printf("Save PCD(size:%d)\n", int(copy_cloud->points.size()));
}

void DepthImage::loadPCDFile(CloudAPtr cloud, 
                             string file_path, 
                             string file_name)
{
    cout<<"Load :" <<file_path<<endl;
    if (pcl::io::loadPCDFile<PointA> (file_path+file_name, *cloud) == -1) //* load the file
        PCL_ERROR ("-----Couldn't read file\n");
    else
        printf("Suscess load size:%d\n", int(cloud->points.size()));
}

// saveImage
void DepthImage::saveImageFile(sensor_msgs::ImageConstPtr image_msg,
                               string file_path,
                               string file_name)
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
    // string file_name = to_string(count);
    cv::imwrite(file_path+file_name+".jpg", image);
    printf("Save Image File (cols:%d rows:%d)\n", image.cols, image.rows);
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "depthimage");

    DepthImage di;
    di.start();

    ros::spin();

    return 0;
}
