#include <ros/ros.h>
#include <std_msgs/Bool.h>

ros::Publisher pub;

void pub_flag()
{
    std_msgs::Bool flag;
    char text[50];
    printf("are you ready?(y/n):");
    scanf("%s", text);

    if(text[0]=='y'){
        flag.data = true;
    }
    else if(text[0]=='n'){
        flag.data = false;
    }
    else{
        flag.data = false;
        printf("prease input y/n\n");
    }
    pub.publish(flag);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_saveflag");
    ros::NodeHandle nh("~");

    pub = nh.advertise<std_msgs::Bool>("/flag", 10);

    ros::Rate rate(20);
    while(ros::ok())
    {
        pub_flag();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
