#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

using namespace std;

class Listener{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;

        string target_frame;
        string source_frame;

    public:
        Listener();
        void callback(const geometry_msgs::TransformConstPtr msg);
};

Listener::Listener()
    : nh("~")
{
    nh.getParam("target_frame", target_frame);
    nh.getParam("source_frame", source_frame);

	sub = nh.subscribe("/transform", 10, &Listener::callback, this);
}

void Listener::callback(const geometry_msgs::TransformConstPtr msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::transformMsgToTF(*msg, transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), target_frame, source_frame));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_broadcast");

    Listener lr;

    ros::spin();

    return 0;
}
