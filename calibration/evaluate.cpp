#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

using namespace std;

ros::Publisher pub_obstacle;
ros::Publisher pub_ground;
ros::Publisher pub_area;

double MIN_X, MAX_X;
double MIN_Y, MAX_Y;
double MIN_Z, MAX_Z;

int grid_dimentions;
double cell_size;
double THRESHOLD;

// PickUp Cloud
template<typename T_ptr>
void pickup_cloud(T_ptr cloud, T_ptr& output)
{
    for(size_t i=0;i<cloud->points.size();i++){
        if( MIN_X < cloud->points[i].x && cloud->points[i].x < MAX_X
            && MIN_Y < cloud->points[i].y && cloud->points[i].y < MAX_Y
            && MIN_Z < cloud->points[i].z && cloud->points[i].z < MAX_Z)
            
            output->points.push_back(cloud->points[i]);
    }
}

// Min Max
template<typename T_ptr>
void constructFullClouds(T_ptr cloud, T_ptr& rm_ground, T_ptr& ground)
{
    float min[grid_dimentions][grid_dimentions];
    float max[grid_dimentions][grid_dimentions];
    bool init[grid_dimentions][grid_dimentions];
    memset(&min,  0, grid_dimentions*grid_dimentions);
    memset(&max,  0, grid_dimentions*grid_dimentions); 
    memset(&init, 0, grid_dimentions*grid_dimentions);

    for(size_t i=0;i<cloud->points.size();i++)
    {
        int x = (grid_dimentions/2)+cloud->points[i].x/cell_size;
        int y = (grid_dimentions/2)+cloud->points[i].y/cell_size;

        if(0<x && x<grid_dimentions && 0<=y && y<grid_dimentions)
        {
            if(!init[x][y]){
                min[x][y] = cloud->points[i].z;
                max[x][y] = cloud->points[i].z;
                init[x][y] = true;
            }
            else{
                min[x][y] = MIN(min[x][y], cloud->points[i].z);
                max[x][y] = MAX(max[x][y], cloud->points[i].z);
            }
        }
    }

    for(size_t i=0;i<cloud->points.size();i++)
    {
        int x = (grid_dimentions/2)+cloud->points[i].x/cell_size;
        int y = (grid_dimentions/2)+cloud->points[i].y/cell_size;
        
        if(0<=x && x<grid_dimentions && 0<=y && y<grid_dimentions)
        {
            if(THRESHOLD<max[x][y]-min[x][y])
                rm_ground->points.push_back(cloud->points[i]);
            else
                ground->points.push_back(cloud->points[i]);
        }
    }
    cout<<"----Remove Ground (Min Max): "<<rm_ground->points.size()<<endl; 
    cout<<"----Ground (Min Max): "<<ground->points.size()<<endl;
}

// evaluate
template<typename T_ptr>
void evaluate(T_ptr cloud)
{
    int size = int(cloud->points.size());
    
    int correct = 0;
    int error = 0;

#pragma omp parallel for
    for(size_t i=0;i<cloud->points.size();i++)
    {
        if(160<cloud->points[i].r && 160<cloud->points[i].g && 160<cloud->points[i].b)
            correct++;
        else
            error++;
    }
    
	for(size_t i=0;i<cloud->points.size();i++)
    	printf("----R:%d G:%df B:%d\n", cloud->points[i].r, cloud->points[i].g, cloud->points[i].b);

    printf("----Size    : %d\n", size);
    printf("----correct : %d %.2f\n", correct, double(correct/size));
    printf("----error   : %d %.2f\n", error,   double(error/size));
}

// Publish PointCloud
template<typename T_ptr>
void CloudPublisher(T_ptr cloud,
                    string target_frame,
                    ros::Publisher pub)
{
     sensor_msgs::PointCloud2 pc2;
     pcl::toROSMsg(*cloud, pc2);
     pc2.header.frame_id = target_frame;
     pc2.header.stamp = ros::Time::now();
     pub.publish(pc2);
}

template<typename T_ptr, typename T_c>
void pcCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	cout<<"Callback"<<endl;

    // Callback PointCloud
    T_ptr input(new T_c);
    pcl::fromROSMsg(*msg, *input);

    // Min Max
    T_ptr obstacle(new T_c);
    T_ptr ground(new T_c);
    constructFullClouds(input, obstacle, ground);

    // Pick Up Cloud
    T_ptr area(new T_c);
    if(0<input->points.size())
        pickup_cloud(input, area);
    
    // evaluate
    if(0<area->points.size())
		evaluate(area);

    // Publish PointCloud
    CloudPublisher(obstacle, msg->header.frame_id, pub_obstacle);
    CloudPublisher(ground,   msg->header.frame_id, pub_ground);
    CloudPublisher(area,     msg->header.frame_id, pub_area);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "evaluate");
    ros::NodeHandle nh("~");

    nh.getParam("min_x", MIN_X);
    nh.getParam("max_x", MAX_X);
    nh.getParam("min_y", MIN_Y);
    nh.getParam("max_y", MAX_Y);
    nh.getParam("min_z", MIN_Z);
    nh.getParam("max_z", MAX_Z);
    nh.getParam("threshold", THRESHOLD);
    nh.getParam("grid_dimentions", grid_dimentions);
    nh.getParam("cell_size", cell_size);

    ros::Subscriber sub = nh.subscribe("/cloud", 10, pcCallback< pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB> >);

    pub_obstacle = nh.advertise<sensor_msgs::PointCloud2>("/obstacle", 10);
    pub_ground   = nh.advertise<sensor_msgs::PointCloud2>("/ground"  , 10);
    pub_area     = nh.advertise<sensor_msgs::PointCloud2>("/area"    , 10);

    ros::spin();

    return 0;
}
