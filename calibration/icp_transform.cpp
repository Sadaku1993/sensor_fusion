#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace sensor_msgs;

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

class icp_transform{
    public:
        icp_transform();
    private:
        void lidar_callback (const PointCloud2ConstPtr& cloud);  
        void camera_callback (const PointCloud2ConstPtr& cloud);
        void transform(const ros::TimerEvent&);

        ros::NodeHandle nh;

        ros::Subscriber sub_lidar;
        ros::Subscriber sub_camera;

        PointCloud2 cloud2_lidar;
        PointCloud2 cloud2_camera;
        
        CloudAPtr pcl_lidar;
        CloudAPtr pcl_camera;

        ros::Timer timer;
};

// コンストラクタ初期化
icp_transform::icp_transform()
    : pcl_lidar(new CloudA), pcl_camera(new CloudA)
{
    sub_lidar  = nh.subscribe<PointCloud2> ("/lidar" , 10, &icp_transform::lidar_callback, this);
    sub_camera = nh.subscribe<PointCloud2> ("/camera", 10, &icp_transform::camera_callback, this);

    timer = nh.createTimer(ros::Duration(1.0), &icp_transform::transform,this);
}

void icp_transform::lidar_callback(const PointCloud2ConstPtr& msg)
{
    cloud2_lidar = *msg;
    pcl::fromROSMsg(*msg, *pcl_lidar);
}

void icp_transform::camera_callback(const PointCloud2ConstPtr& msg)
{
    cloud2_camera = *msg;
    pcl::fromROSMsg(*msg, *pcl_camera);
}

//表示関数
void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

// Matrix to rpy
void calc_rpy(Eigen::Matrix4d matrix){
	tf::Matrix3x3 mat_l;
    double roll, pitch, yaw;
    mat_l.setValue(matrix(0, 0), matrix(0, 1), matrix(0, 2),
                   matrix(1, 0), matrix(1, 1), matrix(1, 2),
                   matrix(2, 0), matrix(2, 1), matrix(2, 2));
	mat_l.getRPY(roll, pitch, yaw, 1);
    printf("Roll Pitch Yaw :\n");
    printf("RPY = < %6.3f, %6.3f. %6.3f >\n\n", roll, pitch, yaw);
}


void icp_transform::transform(const ros::TimerEvent&){
  pcl::IterativeClosestPoint<PointA, PointA> icp;
  
  // icp.setMaxCorrespondenceDistance(0.1);
  // icp.setMaximumIterations(100);
  // icp.setTransformationEpsilon(1e-8);
  // icp.setEuclideanFitnessEpsilon(1e-8);

  icp.setInputTarget(pcl_lidar->makeShared());
  icp.setInputSource(pcl_camera->makeShared());
  
  CloudA Final;
  icp.align(Final);

  //変換matrixを表示する
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
  transformation_matrix = icp.getFinalTransformation ().cast<double>();
  print4x4Matrix (transformation_matrix);
  calc_rpy(transformation_matrix);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_transform");

    icp_transform icp_transform;

    ros::spin();

    return 0;
}
