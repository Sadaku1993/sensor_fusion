/*
    Calc movement from odometry

    author Yudai Sadakuni
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

using namespace std;

class Movement{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher pub;
        double threshold;
        double distance;
        double meter;
        bool flag;
        nav_msgs::Odometry old_odom;
        nav_msgs::Odometry new_odom;

    public:
        Movement();
        void Callback(const nav_msgs::OdometryConstPtr msg);
        void detect();
};

Movement::Movement() 
    : nh("~")
{
    nh.getParam("threshold", threshold);
    sub = nh.subscribe("/odom", 10 , &Movement::Callback, this);
    pub = nh.advertise<std_msgs::Bool>("/stop", 10);
    distance = 0;
    meter = 0;
    flag = false;
}

void Movement::Callback(const nav_msgs::OdometryConstPtr msg)
{
    if(!flag){
        printf("init\n");
        old_odom = *msg;
        flag = true;
    }
    else{
        new_odom = *msg;

        double dt = sqrt( pow((new_odom.pose.pose.position.x - old_odom.pose.pose.position.x), 2) + 
                pow((new_odom.pose.pose.position.y - old_odom.pose.pose.position.y), 2) );
        distance += dt;
		meter += dt;
        printf("dt:%.2f, distance:%.2f meter:%.2f\n", dt, distance, meter); 
        old_odom = *msg;
    }
}

void Movement::detect()
{
    std_msgs::Bool move;
    if(meter < threshold)  
        move.data = false;
    else{
        move.data = true;
        meter = 0;
        cout<<"---Reset---"<<endl;
    }

    pub.publish(move);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calc_movement");

    Movement mv;

    ros::Rate rate(40);
    while(ros::ok())
    {
        mv.detect();
		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}
