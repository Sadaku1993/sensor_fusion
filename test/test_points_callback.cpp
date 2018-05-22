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

pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    sensor_msgs::PointCloud2ConstPtr pc;
    pc = msg;
    pub.publish(pc);
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "test_points_callback");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("cloud/tf", 1, pc_callback);
    pub = n.advertise<sensor_msgs::PointCloud2>("/cloud/test", 1);

    cout<<"start"<<endl;

    ros::spin();

    return 0;
}

