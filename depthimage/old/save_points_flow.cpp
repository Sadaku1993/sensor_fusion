/* save pointcloud 
 *
 * Subscribe:
 *      PointCloud2 (header.frame_id = odom)
 *      Bool optical_flow (stop or move)
 *      Bool WayPoints
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int SAVE_NUM;
int LIDAR_POINTS;

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;

int pc_count = 0;
int t = 0;
bool save = false;

bool stop_ = false;
bool arraival_ = false;

void stop_Callback(const std_msgs::BoolConstPtr msg)
{
    stop_ = msg->data;
}

void arrival_C

int main(int argc, char**argv)
{
    ros::init(argc, argv, "save_points");
    ros::NodeHandle n;

	n.getParam("save_points_flow/save_num", SAVE_NUM);
	n.getParam("save_points_flow/lidar_points", LIDAR_POINTS);

    ros::Subscriber sub = n.subscribe("/cloud", 10, pc_Callback);
    ros::Subscriber sub_flow = n.subscribe("/stop" 10 , stop_Callback);
    ros::Subscriber sub_arrival = n.subscribe("/arrival", 10, arrival_Callback); 

    pcl_save.points.resize(SAVE_NUM*LIDAR_POINTS);
	
    ros::spin();

    return 0;
}
