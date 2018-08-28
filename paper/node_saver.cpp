#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_fusion/NodeInfo.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;

class Saver{
    private:
        // HodeHandle
        ros::NodeHandle nh;
        // pointcloud callback
        ros::Subscriber cloud_sub;
        // camera
        sensor_msgs::ImageConstPtr camera_image;
        sensor_msgs::CameraInfoConstPtr camera_info;
        // save flag
        ros::Subscriber flag_sub;
        // transform
        tf::TransformListener global_listener;
        tf::StampedTransform  global_transform;
        geometry_msgs::Transform tf;
        // frame
        string global_frame;
        string child_frame;
		string laser_frame;
        string camera_frame;
        // publisher
        ros::Publisher pub;
        // init
        int count;
        int save_num;
        int node_num;
        bool is_save;
        string file_path;
        CloudAPtr save_cloud;
        
    public:
        Saver();
        // pointcloud callback
        void cloud_Callback(const sensor_msgs::PointCloud2ConstPtr msg);
        // image callback
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> camera_sync_subs;
        message_filters::Subscriber<sensor_msgs::Image> image_sub;
        message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub;
        message_filters::Synchronizer<camera_sync_subs> camera_sync;
        void camera_Callback(const sensor_msgs::ImageConstPtr image, const sensor_msgs::CameraInfoConstPtr cam_info);
        // bool callback
        void bool_Callback(const std_msgs::BoolConstPtr msg);
        // save process
        void saver(const sensor_msgs::PointCloud2ConstPtr msg);
        void savePCDFile(CloudAPtr cloud, int count);
        void saveNode();
        // transform listener
        void transform_listener();
        // reset params et al.
        void reset();
};

Saver::Saver()
    : nh("~"),
      image_sub(nh, "/image", 10),
      cinfo_sub(nh, "/cinfo", 10),
      camera_sync(camera_sync_subs(10), image_sub, cinfo_sub),
      save_cloud(new CloudA),
      count(0), node_num(0), is_save(false)
{
    // load param
    nh.getParam("save_num", save_num);
    nh.getParam("file_path", file_path);
    nh.getParam("global_frame", global_frame);
	nh.getParam("child_frame" , child_frame);
    nh.getParam("laser_frame" , laser_frame);
    nh.getParam("camera_frame", camera_frame);
    // callback
    cloud_sub = nh.subscribe("/cloud", 10, &Saver::cloud_Callback, this);
	camera_sync.registerCallback(boost::bind(&Saver::camera_Callback, this, _1, _2));
    flag_sub = nh.subscribe("/flag", 10, &Saver::bool_Callback, this);
    //publish
    pub = nh.advertise<sensor_fusion::NodeInfo>("/node", 10);
}

// pointcloud callback
void Saver::cloud_Callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    if(is_save) saver(msg);
}

//camera callback
void Saver::camera_Callback(const sensor_msgs::ImageConstPtr image_msg,
                            const sensor_msgs::CameraInfoConstPtr cinfo_msg)
{
    camera_image = image_msg;
    camera_info  = cinfo_msg;
}

//bool callback
void Saver::bool_Callback(const std_msgs::BoolConstPtr msg){
    is_save = msg->data;
}

// save pointcloud
void Saver::saver(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr input_cloud(new CloudA);
    CloudAPtr threshold_cloud(new CloudA);

    pcl::fromROSMsg(*msg, *input_cloud);
    for(size_t i=0;i<input_cloud->points.size();i++){
        double distance = sqrt(pow(input_cloud->points[i].x, 2)+ pow(input_cloud->points[i].y, 2) + pow(input_cloud->points[i].z, 2));
        if(2.0<distance && distance < 30)
            threshold_cloud->points.push_back(input_cloud->points[i]);
    }

    if(count < save_num){
		*save_cloud += *threshold_cloud;
		if(count % 100 == 0) printf("count:%d/%d Cloud_Size:%d\n", count, save_num, int(save_cloud->points.size()));
	}

	if(count == save_num){
		cout<<"Num:"<<node_num<<" Success Save PointCloud!!! Next Node"<<endl;
		saveNode();
        savePCDFile(save_cloud, node_num);
        node_num++;
		reset();
	}
    
    count++;
}

// save PCD
void Saver::savePCDFile(CloudAPtr cloud, 
                        int count)
{
    CloudAPtr save_cloud(new CloudA);
	pcl::copyPointCloud(*cloud, *save_cloud);
 
	save_cloud->width = 1;
	save_cloud->height = save_cloud->points.size();

    string file_name = to_string(count);
	pcl::io::savePCDFile(file_path+file_name+".pcd", *save_cloud);
    printf("Num:%d saved %d\n", count, int(cloud->points.size()));
}

// save Node
void Saver::saveNode()
{
    sensor_fusion::NodeInfo ninfo;
    ninfo.header.stamp = ros::Time::now();
    ninfo.node = node_num;
    ninfo.image = *camera_image;
    ninfo.cinfo = *camera_info;
	transform_listener();
	ninfo.transform = tf;
    pcl::toROSMsg(*save_cloud, ninfo.cloud);
    ninfo.cloud.header.frame_id = laser_frame;
    pub.publish(ninfo);
}

// transform listener
void Saver::transform_listener()
{
    try{
        ros::Time now = ros::Time::now();
        global_listener.waitForTransform(global_frame, child_frame, now, ros::Duration(1.0));
        global_listener.lookupTransform(global_frame, child_frame,  now, global_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
	tf::transformTFToMsg(global_transform, tf);
}


// reset param
void Saver::reset()
{
    count = 0;
    is_save = false;
    save_cloud->points.clear();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_saver");

    Saver sv;

    ros::spin();

    return 0;
}


