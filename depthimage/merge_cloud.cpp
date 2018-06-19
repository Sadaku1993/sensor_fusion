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

typedef pcl::PointXYZ PointA;
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
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file\n");
    }
    cout<<"Loaded"<<endl;
}

void save(CloudAPtr cloud, int count)
{
    string file_name=to_string(count);
    pcl::io::savePCDFileASCII(FILE_PATH+file_name+".pcd", *cloud);
    printf("saved %d\n", int(cloud->points.size()));
}

void merge()
{
    int size = file_count_boost(FILE_PATH.c_str());
    printf("PCD File Size : %d\n", size);

    for(int i=0;i<size-MERGE_SIZE;i++){
        CloudAPtr merge_cloud;
        for(int j=0;j<MERGE_SIZE;j++){
            CloudAPtr load_cloud;
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
