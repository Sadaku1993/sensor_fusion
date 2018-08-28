#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <iostream>

using namespace std;

class Transform{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;

        tf::TransformListener listener;
        tf::StampedTransform transform;

        string target_frame;
        string source_frame;

    public:
        Transform();
        void mainloop();
};

Transform::Transform()
    : nh("~")
{
    nh.getParam("target_frame", target_frame); // global_frame
    nh.getParam("source_frame", source_frame); // laser_frame
    pub = nh.advertise<geometry_msgs::Transform>("/transform", 10);
}

void Transform::mainloop()
{
    try{
        ros::Time time = ros::Time::now();
        listener.waitForTransform(target_frame, source_frame, time, ros::Duration(0.1));
        listener.lookupTransform(target_frame, source_frame, time, transform);

		geometry_msgs::Transform transform_msg;
		tf::transformTFToMsg(transform, transform_msg);
		pub.publish(transform_msg);

        // SHOW TF
        tf::Vector3 v = transform.getOrigin();
        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();
        tf::Quaternion q = transform.getRotation();
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll ,pitch, yaw);

        printf("x:%.2f y:%.2f z:%.2f roll:%.2f pitch:%.2f yaw:%.2f\n", x, y, z, roll, pitch, yaw);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(0.1).sleep();
    }

}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "transform_publisher");
    
    Transform tf;

    ros::Rate rate(50);

    while(ros::ok())
    {
        tf.mainloop();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
