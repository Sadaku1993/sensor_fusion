#include <ros/ros.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>

#include <sys/stat.h>
#include <sys/types.h>

using namespace std;

class NormalEstimation{
    private:
        ros::NodeHandle nh;

        string FILE_PATH;
        string NORMAL_PATH;
        double search_radius;

    public:
        NormalEstimation();

        int file_count_boost(const boost::filesystem::path& root);

        bool check_node(int count);

        void load(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, int count);
        void save(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, int count);

        void normal_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud_normal);
        void main();
};

NormalEstimation::NormalEstimation()
    : nh("~")
{
    nh.getParam("search_radius", search_radius);
    nh.getParam("FILE_PATH"    , FILE_PATH);
    nh.getParam("NORMAL_PATH" , NORMAL_PATH);
}

int NormalEstimation::file_count_boost(const boost::filesystem::path& root) {
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

bool NormalEstimation::check_node(int count)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    string file_name = FILE_PATH + to_string(count) + ".pcd";
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (file_name, *cloud) == -1) //* load the file
    {
        printf("Node %d is none\n\n", count);
        return false;
    }
    else{
        printf("Node : %d\n", count);
        return true;
    }
}

void NormalEstimation::load(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, int count)
{
    string file_name = FILE_PATH + to_string(count) + ".pcd";
    cout<<"-----Load :" <<file_name<<endl;
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (file_name, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("-----Couldn't read file\n");
    }
}

void NormalEstimation::save(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, int count)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::copyPointCloud(*cloud, *save_cloud);

    save_cloud->width = 1;
    save_cloud->height = save_cloud->points.size();

    string file_name=NORMAL_PATH+to_string(count)+".pcd";

    pcl::io::savePCDFileASCII(file_name, *save_cloud);
    cout<<"-----Save :" <<file_name <<endl;
}

void NormalEstimation::normal_estimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud_normal)
{
	cout<<"-----Normal Estimation"<<endl;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (search_radius);
    ne.compute(*normals);

    pcl::PointXYZRGBNormal tmp;
    for(size_t i=0;i<cloud->points.size();i++)
    {
		tmp.x = cloud->points[i].x;
		tmp.y = cloud->points[i].y;
		tmp.z = cloud->points[i].z;
		tmp.r = cloud->points[i].r;
		tmp.g = cloud->points[i].g;
		tmp.b = cloud->points[i].b;
		if(!isnan(normals->points[i].normal_x)){
			tmp.normal_x = normals->points[i].normal_x;
		}
		else{
			tmp.normal_x = 0.0;
		}
		if(!isnan(normals->points[i].normal_y)){
			tmp.normal_y = normals->points[i].normal_y;
		}
		else{
			tmp.normal_y = 0.0;
		}
		if(!isnan(normals->points[i].normal_z)){
			tmp.normal_z = normals->points[i].normal_z;
		}
		else{
			tmp.normal_z = 0.0;
		}
		if(!isnan(normals->points[i].curvature)){
			tmp.curvature = normals->points[i].curvature;
		}
		else{
			tmp.curvature = 0.0;
		}
		cloud_normal->points.push_back(tmp);
	}
}

void NormalEstimation::main()
{
    int size = file_count_boost(FILE_PATH.c_str());
    printf("PCD File Size : %d\n", size);

    for(int i=0;i<size;i++)
    {
        if(!check_node(i)) continue;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr load_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        load(load_cloud, i);
        normal_estimation(load_cloud, normal_cloud);
        save(normal_cloud, i);
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "normal_estimation_local");
    
    NormalEstimation ne;

    ne.main();

    return 0;
}
