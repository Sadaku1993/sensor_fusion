/*

author : Yudai Sadakuni

division sqlidar pointcloud

*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

typedef pcl::PointXYZRGB PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

ros::Publisher pub;

void callback(const PointCloud2ConstPtr& center_msg,
			  const PointCloud2ConstPtr& right_msg,
			  const PointCloud2ConstPtr& left_msg)
{
	cout<<"ALL GREEN"<<endl;
	CloudAPtr center_cloud (new CloudA);
	CloudAPtr right_cloud (new CloudA);
	CloudAPtr left_cloud (new CloudA);

	pcl::fromROSMsg(*center_msg, *center_cloud);
	pcl::fromROSMsg(*right_msg,  *right_cloud);
	pcl::fromROSMsg(*left_msg,   *left_cloud);

	CloudAPtr integrate_cloud (new CloudA);

	*integrate_cloud += *center_cloud;
	*integrate_cloud += *right_cloud;
	*integrate_cloud += *left_cloud;
	
	if(0<integrate_cloud->points.size()){
		PointCloud2 output;
		pcl::toROSMsg(*integrate_cloud, output);
		output.header.frame_id = "/centerlaser";
		output.header.stamp = ros::Time::now();
		pub.publish(output);
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "integration");
    ros::NodeHandle nh;

	message_filters::Subscriber<PointCloud2> zed0(nh, "/sq_lidar/points/center", 10);
	message_filters::Subscriber<PointCloud2> zed1(nh, "/sq_lidar/points/right" , 10);
	message_filters::Subscriber<PointCloud2> zed2(nh, "/sq_lidar/points/left"  , 10);
	
	typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2, PointCloud2> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), zed0, zed1, zed2);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3));
	
	// TimeSynchronizer<PointCloud2, PointCloud2, PointCloud2> sync(zed0, zed1, zed2, 10);
	// sync.registerCallback(boost::bind(&callback, _1, _2, _3));
	
	pub = nh.advertise<sensor_msgs::PointCloud2>("/sq_lidar/points/integrate", 1);

	ros::spin();

    return 0;
}
