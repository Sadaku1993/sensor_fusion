#include <ros/ros.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>

#include <sys/stat.h>
#include <sys/types.h>

using namespace std;

class Integrate{
    private:
        ros::NodeHandle nh;

        string FILE_PATH;
        string INTEGRATRE_PATH;
        string MAP_NAME;

    public:
        Integrate();
        
        int file_count_boost(const boost::filesystem::path& root);
        void load(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, int count);
        void save(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
        void main();
};

Integrate::Integrate()
    : nh("~")
{
    nh.getParam("FILE_PATH", FILE_PATH);
    nh.getParam("INTEGRATE_PATH", INTEGRATRE_PATH);
    nh.getParam("MAP_NAME", MAP_NAME);
}

int Integrate::file_count_boost(const boost::filesystem::path& root) {
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

void Integrate::load(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, int count)
{
    string file_name = FILE_PATH + to_string(count) + ".pcd";
    cout<<"-----Load :" <<file_name<<endl;
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (file_name, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("-----Couldn't read file\n");
    }
}

void Integrate::save(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::copyPointCloud(*cloud, *save_cloud);

    save_cloud->width = 1;
    save_cloud->height = save_cloud->points.size();

    string file_name=INTEGRATRE_PATH+MAP_NAME;

    pcl::io::savePCDFileASCII(file_name, *save_cloud);
    cout<<"-----Save :" <<file_name <<endl;
}


void Integrate::main()
{
    int size = file_count_boost(FILE_PATH.c_str());
    printf("PCD File Size : %d\n", size);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr integrate_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    for(int i=0;i<size;i++)
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        load(cloud, i);
        *integrate_cloud += *cloud;
    }

    save(integrate_cloud);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_integrater");

    Integrate in;

    in.main();

    return 0;
}



























