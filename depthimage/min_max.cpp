#include <ros/ros.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <sys/stat.h>
#include <sys/types.h>

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

typedef pcl::PointXYZRGBNormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;

class MIN_MAX{
    private:
        ros::NodeHandle nh;

        string FILE_PATH;
        string RM_GROUND_PATH;
        string GROUND_PATH;

        int grid_dimentions;
        double cell_size;
        double height_threshold;

    public:
        MIN_MAX();

        int file_count_boost(const boost::filesystem::path& root);
        bool check_node(int count);

        void load(CloudAPtr& cloud, int count);
        void save(CloudAPtr cloud, string file_path,int count);

        void constructFullClouds(CloudAPtr cloud, CloudAPtr& obstacle, CloudAPtr& ground);
        
        void main();
};


MIN_MAX::MIN_MAX()
    : nh("~")
{
    nh.getParam("FILE_PATH",        FILE_PATH);
    nh.getParam("RM_GROUND_PATH",   RM_GROUND_PATH);
    nh.getParam("GROUND_PATH",      GROUND_PATH);
    nh.getParam("grid_dimentions",  grid_dimentions);
    nh.getParam("cell_size",        cell_size);
    nh.getParam("height_threshold", height_threshold);
}

int MIN_MAX::file_count_boost(const boost::filesystem::path& root) {
    namespace fs = boost::filesystem;
    if (!fs::exists(root) || !fs::is_directory(root)) return 0;
    int result = 0;
    fs::directory_iterator last;
    for (fs::directory_iterator pos(root); pos != last; ++pos) {
        ++result;
        if (fs::is_directory(*pos)) result += file_count_boost(pos->path());
    }
    return result;
}

bool MIN_MAX::check_node(int count)
{
    CloudAPtr cloud(new CloudA);

    string file_name = FILE_PATH + to_string(count) + ".pcd";
    
    if (pcl::io::loadPCDFile<PointA> (file_name, *cloud) == -1) //* load the file
    {
        printf("Node %d is none\n\n", count);
        return false;
    }
    else{
        printf("Node : %d\n", count);
        return true;
    }
}

void MIN_MAX::load(CloudAPtr& cloud, int count)
{
    string file_name = FILE_PATH + to_string(count) + ".pcd";
    cout<<"----Load :" <<file_name<<endl;
    
    if (pcl::io::loadPCDFile<PointA> (file_name, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("----Couldn't read file\n");
    }
}

void MIN_MAX::save(CloudAPtr cloud, string file_path, int count)
{
    CloudAPtr save_cloud(new CloudA);
    pcl::copyPointCloud(*cloud, *save_cloud);

    save_cloud->width = 1;
    save_cloud->height = save_cloud->points.size();

    string file_name=file_path+to_string(count)+".pcd";

    pcl::io::savePCDFileASCII(file_name, *save_cloud);
    cout<<"----Save :" <<file_name <<endl;
}


void MIN_MAX::constructFullClouds(CloudAPtr cloud, CloudAPtr& rm_ground, CloudAPtr& ground)
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
            if(height_threshold<max[x][y]-min[x][y])
                rm_ground->points.push_back(cloud->points[i]);
            else
                ground->points.push_back(cloud->points[i]);
        }
    }
    cout<<"----Remove Ground (Min Max): "<<rm_ground->points.size()<<endl; 
    cout<<"----Ground (Min Max): "<<ground->points.size()<<endl;
}


void MIN_MAX::main()
{
    int size = file_count_boost(FILE_PATH.c_str());
    printf("PCD File Size : %d\n", size);

    for(int i=0;i<size;i++)
    {
        if(!check_node(i)) continue;

        CloudAPtr load_cloud(new CloudA);
        CloudAPtr rm_ground(new CloudA);
        CloudAPtr ground(new CloudA);

        load(load_cloud, i);
        constructFullClouds(load_cloud, rm_ground, ground);
        save(rm_ground, RM_GROUND_PATH, i);
        save(ground, GROUND_PATH, i);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "normal_estimation_local");

    MIN_MAX min_max;
    

    min_max.main();

    return 0;
}
