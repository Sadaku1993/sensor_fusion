#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <time.h>

using namespace std;

typedef pcl::PointXYZI PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

class SaveCloud{
    private:
        ros::NodeHandle nh;
        ros::Subscriber bool_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber cloud_sub;
        
        // 0:Saving 1:Success 2:Fail
        ros::Publisher info_pub;
        ros::Publisher cloud_pub;

        CloudAPtr save_cloud;
        CloudAPtr old_cloud;

        nav_msgs::Odometry init_odom;
        nav_msgs::Odometry odom_;

        Eigen::Matrix4f transform_matrix;
        Eigen::Matrix4f inverse_transform_matrix;

        int count;
        int save_count;

        bool start_flag;
        bool finish_flag;

    public:
        SaveCloud();
        void boolCallback(const std_msgs::BoolConstPtr msg);
        void odomCallback(const nav_msgs::OdometryConstPtr msg);
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr msg);
        Eigen::Matrix4f create_matrix(nav_msgs::Odometry odom_now, float reflect);
};

SaveCloud::SaveCloud()
    : nh("~"), save_cloud(new CloudA), old_cloud(new CloudA)
{
    nh.getParam("save_cloud", save_count);
    bool_sub  = nh.subscribe("/start" ,10, &SaveCloud::boolCallback,  this);
    odom_sub  = nh.subscribe("/odom" , 10, &SaveCloud::odomCallback,  this);
    cloud_sub = nh.subscribe("/cloud", 10, &SaveCloud::cloudCallback, this);
    
    info_pub = nh.advertise<std_msgs::Int32>("/info", 10);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud/lcl", 10);

    init_odom.header.frame_id = "/odom";
    init_odom.child_frame_id = "/base_link";
    init_odom.pose.pose.position.x = 0.0;
    init_odom.pose.pose.position.y = 0.0;
    init_odom.pose.pose.position.z = 0.0;
    init_odom.pose.pose.orientation.x = 0.0;
    init_odom.pose.pose.orientation.y = 0.0;
    init_odom.pose.pose.orientation.z = 0.0;
    init_odom.pose.pose.orientation.w = 0.0;
    odom_ = init_odom;

    count = 0;
    start_flag = false;
    finish_flag = false;
}

Eigen::Matrix4f SaveCloud::create_matrix(nav_msgs::Odometry odom_now, float reflect){
    double roll_now, pitch_now, yaw_now;
    tf::Quaternion q_now(odom_now.pose.pose.orientation.x, odom_now.pose.pose.orientation.y, odom_now.pose.pose.orientation.z, odom_now.pose.pose.orientation.w);
    tf::Matrix3x3(q_now).getRPY(roll_now, pitch_now, yaw_now);
    Eigen::Translation3f init_translation(reflect*odom_now.pose.pose.position.x, reflect*odom_now.pose.pose.position.y, reflect*odom_now.pose.pose.position.z);
    Eigen::AngleAxisf init_rotation_x(reflect*roll_now, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(reflect*pitch_now, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(reflect*yaw_now, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
    return init_guess;
}

void SaveCloud::boolCallback(const std_msgs::BoolConstPtr msg)
{
    start_flag = msg->data;
}

void SaveCloud::odomCallback(const nav_msgs::OdometryConstPtr msg)
{
    odom_ = *msg;
    transform_matrix = create_matrix(odom_, 1.0);
}

void SaveCloud::cloudCallback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    std_msgs::Int32 info;

    if(start_flag && !finish_flag)
    {
        CloudAPtr input_cloud(new CloudA);
        CloudAPtr transform_cloud(new CloudA);
        CloudAPtr threshold_cloud(new CloudA);
        CloudAPtr inverse_cloud(new CloudA);

        pcl::fromROSMsg(*msg, *input_cloud);
        pcl::transformPointCloud(*input_cloud, *transform_cloud, transform_matrix);

        for(size_t i=0;i<input_cloud->points.size();i++){
            double distance = sqrt(pow(input_cloud->points[i].x, 2)+
                    pow(input_cloud->points[i].y, 2)+
                    pow(input_cloud->points[i].z, 2));
            if(distance < 30)
                threshold_cloud->points.push_back(transform_cloud->points[i]);
        }

        // Eigen::Matrix4f inverse_transform_matrix = transform_matrix.inverse();
        // pcl::transformPointCloud(*threshold_cloud, *inverse_cloud, inverse_transform_matrix);
        // *save_cloud += *inverse_cloud;

        *save_cloud = *threshold_cloud;

        if(count == save_count){
            sensor_msgs::PointCloud2 pc2;
            pcl::toROSMsg(*save_cloud, pc2);
            // pc2.header.frame_id = msg->header.frame_id;  
            pc2.header.frame_id = init_odom.header.frame_id;
            pc2.header.stamp = ros::Time::now();
            cloud_pub.publish(pc2);

            save_cloud->points.clear();

            info.data = 1;
            count = 0;
            finish_flag = true;

            cout<<"Finish Save Cloud"<<endl;
        }
        else{
            info.data = 0;
            cout<<"Saving Cloud"<<endl;
        }

        count++;
    }
    else{
        info.data = 2;
        cout<<"Waiting Arrive WayPoint"<<endl;
    }

    info_pub.publish(info);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lcl_for_depthimage");

    SaveCloud savecloud;

    ros::spin();

    return 0;
}
