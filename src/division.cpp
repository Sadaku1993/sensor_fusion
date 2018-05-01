/*

author : Yudai Sadakuni

division sqlidar pointcloud

*/

#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/point_cloud_conversion.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))
#define cell_size 0.3
#define full_clouds true
#define grid_dimentions 140

ros::Publisher pub_center;
ros::Publisher pub_left;
ros::Publisher pub_right;

bool pc_ = false;
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_(new pcl::PointCloud<pcl::PointXYZ>);
void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pc_ = true;
    fromROSMsg(*msg,*pcl_);
}

void area(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ> pcl_center;
    pcl::PointCloud<pcl::PointXYZ> pcl_left;
    pcl::PointCloud<pcl::PointXYZ> pcl_right;

    int size = cloud->size();
    printf("size:%d\n",size);

    for(int i=0;i<size;i++)
    {
        int x = ((grid_dimentions/2)+cloud->points[i].x/cell_size);
        int y = ((grid_dimentions/2)+cloud->points[i].y/cell_size);

        if( 0<=x && x<=grid_dimentions && 0<=y && y<=grid_dimentions){
            if(grid_dimentions/2<=y && sqrt(3)*x+grid_dimentions/2*(1-sqrt(3))<=y) pcl_left.push_back(cloud->points[i]);

            else if(y<=grid_dimentions/2 && y<=(-1)*sqrt(3)*x+(1+sqrt(3))*grid_dimentions/2) pcl_right.push_back(cloud->points[i]);

            else pcl_center.push_back(cloud->points[i]);
            
        }
    }

    sensor_msgs::PointCloud2 pc_center;
    pcl::toROSMsg(pcl_center, pc_center);
    pc_center.header.stamp = ros::Time::now();
    pc_center.header.frame_id = "/centerlaser";
    pub_center.publish(pc_center);

    sensor_msgs::PointCloud2 pc_left;
    pcl::toROSMsg(pcl_left, pc_left);
    pc_left.header.stamp = ros::Time::now();
    pc_left.header.frame_id = "/centerlaser";
    pub_left.publish(pc_left);

    sensor_msgs::PointCloud2 pc_right;
    pcl::toROSMsg(pcl_right, pc_right);
    pc_right.header.stamp = ros::Time::now();
    pc_right.header.frame_id = "/centerlaser";
    pub_right.publish(pc_right);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "zed_bridge");
    ros::NodeHandle n;

    ros::Subscriber pc_callback = n.subscribe("/sq_lidar/points/tf",10,pcCallback);

    pub_center = n.advertise<sensor_msgs::PointCloud2>("/sq_lidar/points/center",10);
    pub_left   = n.advertise<sensor_msgs::PointCloud2>("/sq_lidar/points/left",10);
    pub_right  = n.advertise<sensor_msgs::PointCloud2>("/sq_lidar/points/right",10);

    ros::Rate rate(20);
    while(ros::ok())
    {
        if(pc_) area(pcl_);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
