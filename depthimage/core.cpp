/*
    WayPointに到達したか,をcalc_movementから判定し,
    JoyInterruptにデータ収集のための停止Flagを送る.
*/

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <time.h>

using namespace std;

class Core{
    private:
        ros::NodeHandle nh;
        ros::Subscriber waypoint_sub;
        ros::Subscriber info_sub;

        ros::Publisher stop_pub;
        ros::Publisher save_pub;
        
        std_msgs::Bool stop_flag;
        std_msgs::Bool save_flag;

        bool waypoint_flag;
        int info;

        double timer;
        time_t old_time;
        time_t new_time;
    public:
        Core();
        void waypointCallback(const std_msgs::BoolConstPtr msg);
        void infoCallback(const std_msgs::Int32ConstPtr msg);
        void save();
        
};

Core::Core()
    : nh("~")
{
    nh.getParam("timer", timer);
    
    waypoint_sub = nh.subscribe("/waypoint", 10, &Core::waypointCallback, this);
    info_sub = nh.subscribe("/info", 10, &Core::infoCallback, this);

    stop_pub = nh.advertise<std_msgs::Bool>("/stop", 10);
    save_pub = nh.advertise<std_msgs::Bool>("/save", 10);

    time(&old_time);
    time(&new_time);

    waypoint_flag = false;
    info = 2;
}

void Core::waypointCallback(const std_msgs::BoolConstPtr msg)
{
    waypoint_flag = msg->data;

    // WayPointに到達した
    if(waypoint_flag){
        printf("Start Optical Flow and Save PointCloud\n");
        
        // 時間を更新
        time(&old_time);
        time(&new_time);
        save();
        printf("finish\n");
    }
    else{
        printf("Not Arrive WayPoint\n");
        stop_flag.data = false;
        stop_pub.publish(stop_flag);
    }
}

void Core::infoCallback(const std_msgs::Int32ConstPtr msg)
{
    info = msg->data;
    if(info == 0)
        cout<<"Saving Cloud (lcl)"<<endl;
    else if(info == 1)
        cout<<"Finish Save Cloud (lcl)"<<endl;
    else if(info == 2)
        cout<<"Waiting Arrive WayPoint (lcl)"<<endl;
}

/*
void Core::save()
{
    while(difftime(new_time, old_time) < timer)
    {
        printf("timer:%.2f diff_time:%.2f\n", timer, difftime(new_time, old_time));
        joy_flag.data = true;
        joy_pub.publish(joy_flag);

        time(&new_time);
    }
}
*/

void Core::save()
{
    while(true)
    {
        // joystick interrupt用
        stop_flag.data = true;
        stop_pub.publish(stop_flag);

        // lcl用
        save_flag.data = true;
        save_pub.publish(save_flag);

        if(info == 0)
            break;
    }
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "core");

    Core co;
    
    ros::spin();
    return 0;
}
