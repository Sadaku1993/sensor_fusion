/*
    WayPointに到達したか,をcalc_movementから判定し,
    JoyInterruptにデータ収集のための停止Flagを送る.
*/

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <time.h>

using namespace std;

class Core{
    private:
        ros::NodeHandle nh;
        ros::Subscriber movement_sub;
        ros::Publisher  joy_pub;
        std_msgs::Bool joy_flag;
        double timer;
        time_t old_time;
        time_t new_time;
    public:
        Core();
        void moveCallback(const std_msgs::BoolConstPtr msg);
        void save();
};

Core::Core()
    : nh("~")
{
    nh.getParam("timer", timer);
    movement_sub = nh.subscribe("/stop", 10, &Core::moveCallback, this);
    joy_pub = nh.advertise<std_msgs::Bool>("/bool", 10);
    time(&old_time);
    time(&new_time);
}

void Core::moveCallback(const std_msgs::BoolConstPtr msg)
{
    // WayPointに到達した
    if(msg->data){
        printf("Start Optical Flow and Save PointCloud\n");
        
        // 時間を更新
        time(&old_time);
        time(&new_time);
        save();
        printf("finish\n");
    }
    else{
        printf("Not Arrive WayPoint\n");
        joy_flag.data = false;
        joy_pub.publish(joy_flag);
    }
}

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




int main(int argc, char**argv)
{
    ros::init(argc, argv, "core");

    Core co;
    
    ros::spin();
    return 0;
}
