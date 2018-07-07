/*
    depthimage_creater

    自律走行により取得した画像, 点群データ, 地図を元に
    DepthImageを自動作成する

    Subscribe:
        node (sensor_fusion_msgs)
        transform
        MAP(PCD)

    Advertise:
        DepthImage
    
    Author: Yudai Sadakuni
*/

#include <sensor_fusion/depthimage_creater.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depthimage_creater");

    DepthImage di;

    di.main();

    ros::spin();

    return 0;
}
