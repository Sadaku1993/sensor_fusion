// #include <pcl/point_cloud.h>
// #include <pcl/io/pcd_io.h>
// 
// typedef pcl::PointXYZI PointA;
// typedef pcl::PointCloud<PointA> CloudA;
// typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;
// 
// using namespace std;

void savePCDFile(CloudAPtr cloud, int count)
{
    string file_name = to_string(count);
    // string path = HOME_DIRS + "/" + FILE_PATH + "/" + SAVE_PATH;
    pcl::io::savePCDFileASCII("/home/amsl/PCD/Save/"+file_name+".pcd", *cloud);
    printf("Num:%d saved %d\n", count, int(cloud->points.size()));
}

