/*
test code for subscribe pointcloud

Subscribe:
    PointCloud2
Publish:
    PointCloud2

author:Yudai Sadakuni

*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

ros::Publisher pub_;

void pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    sensor_msgs::PointCloud2 pc;
    pc = *msg;
    pub_.publish(pc);
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "test_points_callback");
    ros::NodeHandle n;

    ros::Subscriber sub_ = n.subscribe("cloud/tf", 10, pc_callback);
    pub_ = n.advertise<sensor_msgs::PointCloud2>("/cloud/test", 10);

    cout<<"start"<<endl;

    ros::spin();

    return 0;
}

