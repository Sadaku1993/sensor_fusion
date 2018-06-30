#include <sensor_fusion/save_data.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_data");

    SaveData sv;

    ros::spin();

    return 0;
}
