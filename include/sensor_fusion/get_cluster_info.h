#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
using namespace std;
using namespace Eigen;

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

class Clustering{
    private:
        struct Cluster{
            float x; 
            float y; 
            float z;
            float width;
            float hieght;
            float depth;
            float curvature;
            Vector3f min_p;
            Vector3f max_p;
        };

    public:
        Clustering();
        void clustering(CloudAPtr pt, CloudA cloud);
        void getClusterInfo(CloudA pt,Cluster& cluster);
        bool detection(Cluster cluster, CloudAPtr pt, CloudA& cloud);
};

void Clustering::clustering(CloudAPtr cloud_in, CloudA cloud){
    //downsampled point's z =>0
    vector<float> tmp_z;
    tmp_z.resize(cloud_in->points.size());
	for(int i=0;i<(int)cloud_in->points.size();i++){
        tmp_z[i]=cloud_in->points[i].z;
		cloud_in->points[i].z  = 0.0;
    }
    //Clustering//
    pcl::search::KdTree<PointA>::Ptr tree (new pcl::search::KdTree<PointA>);
    tree->setInputCloud (cloud_in);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointA> ec;
    ec.setClusterTolerance (0.05); // 15cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (2000);
    ec.setSearchMethod (tree);
    ec.setInputCloud(cloud_in);
    ec.extract (cluster_indices);
    //reset z value
	for(int i=0;i<(int)cloud_in->points.size();i++)
        cloud_in->points[i].z=tmp_z[i];

    bool flag = false;
    CloudA front_cloud;
    for(int iii=0;iii<(int)cluster_indices.size();iii++)
    {
        // cluster points
        CloudAPtr cloud_cluster (new CloudA);
        cloud_cluster->points.resize(cluster_indices[iii].indices.size());
        // cluster data
        Cluster data;
        for(int jjj=0;jjj<int(cluster_indices[iii].indices.size());jjj++){
            int p_num = cluster_indices[iii].indices[jjj];
            cloud_cluster->points[jjj] = cloud_in->points[p_num];
        }
        Clustering::getClusterInfo(*cloud_cluster, data);
        Clustering::detection(data, *cloud_cluster);
        flag = detection(data, *cloud_cluster, front_cloud);
        if(flag) break;
    }
}

void Clustering::getClusterInfo(CloudA pt, Cluster& cluster)
{
    Vector3f centroid;
    centroid[0]=pt.points[0].x;
    centroid[1]=pt.points[0].y;
    centroid[2]=pt.points[0].z;

    Vector3f min_p;
    min_p[0]=pt.points[0].x;
    min_p[1]=pt.points[0].y;
    min_p[2]=pt.points[0].z;

    Vector3f max_p;
    max_p[0]=pt.points[0].x;
    max_p[1]=pt.points[0].y;
    max_p[2]=pt.points[0].z;

    for(size_t i=1;i<pt.points.size();i++){
        centroid[0]+=pt.points[i].x;
        centroid[1]+=pt.points[i].y;
        centroid[2]+=pt.points[i].z;
        if (pt.points[i].x<min_p[0]) min_p[0]=pt.points[i].x;
        if (pt.points[i].y<min_p[1]) min_p[1]=pt.points[i].y;
        if (pt.points[i].z<min_p[2]) min_p[2]=pt.points[i].z;

        if (pt.points[i].x>max_p[0]) max_p[0]=pt.points[i].x;
        if (pt.points[i].y>max_p[1]) max_p[1]=pt.points[i].y;
        if (pt.points[i].z>max_p[2]) max_p[2]=pt.points[i].z;
    }

    cluster.x=centroid[0]/(float)pt.points.size();
    cluster.y=centroid[1]/(float)pt.points.size();
    cluster.z=centroid[2]/(float)pt.points.size();
    cluster.depth  = max_p[0]-min_p[0];
    cluster.width  = max_p[1]-min_p[1];
    cluster.height = max_p[2]-min_p[2]; 
    cluster.min_p = min_p;
    cluster.max_p = max_p;
}

bool Clustering::detection(Cluster cluster,
                           CloudAPtr pt,
                           CloudA& cloud)
{
    bool detect = false;
    
    // キャリブレーションボードが正面かつサイズが正確に検出できているか
    if(abs(cluster.y) < 0.1 &&
       abs(cluster.width - 1.0) < 0.1 && 
       abs(cluster.height - 1.0) < 0.1){
        cloud = *pt;
        detect = true;
    }
    else{
        detect = false;
    }
    return detect;
}
