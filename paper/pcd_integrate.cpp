#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <sensor_fusion/NodeInfo.h>

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;
using namespace Eigen;

class DepthImage{
    private:
        // NodeHandler
        ros::NodeHandle nh;
        // Callback
        ros::Subscriber node_sub;
        // Frame
        string global_frame;
        string laser_frame;
        // transform
        tf::TransformListener global_listener;
        tf::StampedTransform  global_transform;
        // cloud
        CloudAPtr save_cloud;
        // node num
        int min_node;
        int max_node;
        // save path
        string file_path;
 
    public:
        DepthImage();

        void nodeCallback(const sensor_fusion::NodeInfoConstPtr msg);

        void transform_pointcloud(CloudAPtr cloud,
                                  CloudAPtr& trans_cloud,  
                                  tf::StampedTransform transform,
                                  string target_frame,
                                  string source_frame);

        void savePCDFile(CloudAPtr cloud, string name);
};

        
DepthImage::DepthImage()
    : nh("~"), save_cloud(new CloudA)
{
    nh.getParam("global_frame", global_frame);
    nh.getParam("laser_frame" , laser_frame);
    nh.getParam("file_path"   , file_path);

    nh.param<int>("min_node", min_node, 0);
    nh.param<int>("max_node", max_node, 0);

    // callback
    node_sub = nh.subscribe("/node", 10, &DepthImage::nodeCallback, this);
}


void DepthImage::nodeCallback(const sensor_fusion::NodeInfoConstPtr msg)
{
    printf("Node:%3d\n", int(msg->node));
    try{
        ros::Time time = ros::Time::now();
        global_listener.waitForTransform(global_frame, laser_frame, time, ros::Duration(1.0));
        global_listener.lookupTransform(global_frame, laser_frame, time, global_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
    }

    // transform pointcloud (laser_frame --> global_frame)
    CloudAPtr cloud(new CloudA);
    CloudAPtr cloud_global(new CloudA);
    pcl::fromROSMsg(msg->cloud, *cloud);
    transform_pointcloud(cloud,
                         cloud_global,
                         global_transform,
                         global_frame,
                         laser_frame);

    // save pointcloud
    if(min_node<=msg->node || msg->node<=max_node){
        *save_cloud += *cloud_global;
        printf("   size:%d\n", int(cloud_global->points.size()));
    }

    if(msg->node == max_node)
    {
        savePCDFile(save_cloud, "map");
        return;
    }
}

// transform_pointclud
void DepthImage::transform_pointcloud(CloudAPtr cloud,
                                      CloudAPtr &trans_cloud,
                                      tf::StampedTransform stamped_transform,
                                      string target_frame,
                                      string source_frame)
{
    tf::Transform transform;
    double x = stamped_transform.getOrigin().x();
    double y = stamped_transform.getOrigin().y();
    double z = stamped_transform.getOrigin().z();
    double q_x = stamped_transform.getRotation().x();
    double q_y = stamped_transform.getRotation().y();
    double q_z = stamped_transform.getRotation().z();
    double q_w = stamped_transform.getRotation().w();
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(q_x, q_y, q_z, q_w));

    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(q_x, q_y, q_z, q_w)).getRPY(roll ,pitch, yaw);

    pcl_ros::transformPointCloud(*cloud, *trans_cloud, transform);
    trans_cloud->header.frame_id = target_frame;
    
    cout<<"   "<<target_frame<<"-->"<<source_frame
        <<" x:"<<x<<" y:"<<y<<" z:"<<z<<" roll:"<<roll<<" pitch:"<<pitch<<" yaw:"<<yaw<<endl;

}

void DepthImage::savePCDFile(CloudAPtr cloud, string name)
{
    CloudAPtr copy_cloud(new CloudA);
    pcl::copyPointCloud(*cloud, *copy_cloud);

    copy_cloud->width = 1;
    copy_cloud->height = copy_cloud->points.size();

    pcl::io::savePCDFile(file_path+name+".pcd", *copy_cloud);
    printf("Save PCD(size:%d)\n", int(copy_cloud->points.size()));
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_integrater");

    DepthImage di;

    ros::spin();

    return 0;
}
