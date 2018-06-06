#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (X) : (y))

#define cell_size 0.1
#define grid_dimentions 60
#define min_threshold 0.20
#define max_threshold 3.0

ros::Publisher pub;

using namespace std;

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

void constructFullClouds(CloudAPtr cloud, CloudAPtr& rm_ground){
    
    float min[grid_dimentions][grid_dimentions];
    float max[grid_dimentions][grid_dimentions];
    bool init[grid_dimentions][grid_dimentions];

    memset(&min,  0, grid_dimentions*grid_dimentions);
    memset(&max,  0, grid_dimentions*grid_dimentions); 
    memset(&init, 0, grid_dimentions*grid_dimentions);

#pragma omp parallel for
    for(size_t i=0;i<cloud->points.size();i++)
    {
        int x = (grid_dimentions/2)+cloud->points[i].x/cell_size;
        int y = (grid_dimentions/2)+cloud->points[i].y/cell_size;

        if(x<=0 && x<grid_dimentions && 
           0<=0 && y<grid_dimentions)
        {
            if(!init[x][y]){
                min[x][y] = cloud->points[i].z;
                max[x][y] = cloud->points[i].z;
                init[x][y] = true;
            }
            else{
                min[x][y] = cloud->points[i].z;
                max[x][y] = cloud->points[i].z;
            }
        }
    }

#pragma omp parallel for
    for(size_t i=0;i<cloud->points.size();i++)
    {
        int x = (grid_dimentions/2)+cloud->points[i].x/cell_size;
        int y = (grid_dimentions/2)+cloud->points[i].y/cell_size;
        
        if(0<=x && grid_dimentions<x &&
           0<=y && grid_dimentions<y)
        {
            if(min_threshold<min[x][y] && max[x][y]<max_threshold)
                rm_ground->points.push_back(cloud->points[i]);
        }
    }
}
                         

void Callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "sq_rm_ground_min_max");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cloud", 10, Callback);
    pub = n.advertise<sensor_msgs::PointCloud2>("/rm_ground", 10);

    ros::spin();

    return 0;
}
