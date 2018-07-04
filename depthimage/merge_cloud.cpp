
/*
 * Concatenete PointCloud
 *
 *
 */

#include <ros/ros.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <string>
#include <boost/filesystem.hpp>

#include <sys/stat.h>
#include <sys/types.h>

using namespace std;

typedef pcl::PointXYZRGBNormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

string FILE_PATH = "/home/amsl/PCD/Save/";
string MERGE_PATH = "/home/amsl/PCD/Merge/";

int MERGE_SIZE = 5;

int file_count_boost(const boost::filesystem::path& root) {
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

void load(CloudAPtr& cloud, int count)
{
    string file_name = FILE_PATH + to_string(count) + ".pcd";
    cout<<"-----Load :" <<file_name<<endl;
    
    if (pcl::io::loadPCDFile<PointA> (file_name, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("-----Couldn't read file\n");
    }
}

bool check_node(int count)
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


void save(CloudAPtr cloud, int count)
{
    string file_name=to_string(count);
    pcl::io::savePCDFileASCII(MERGE_PATH+file_name+".pcd", *cloud);
    printf("---------Save:%d Size: %d\n\n", count, int(cloud->points.size()));
}

void merge()
{
    int size = file_count_boost(FILE_PATH.c_str());
    printf("PCD File Size : %d\n", size);

    for(int i=0;i<size-MERGE_SIZE;i++){
        
        if(!check_node(i)) continue;

        CloudAPtr merge_cloud(new CloudA);
        for(int j=0;j<MERGE_SIZE;j++){
            CloudAPtr load_cloud(new CloudA);
            load(load_cloud, i+j);
            *merge_cloud += *load_cloud;
        }
        save(merge_cloud, i);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "merge_cloud");
    ros::NodeHandle n;

    merge();

    return 0;
}
