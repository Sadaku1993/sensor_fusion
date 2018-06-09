#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sensor_fusion/Cluster.h>
#include <sensor_fusion/ClusterArray.h>

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA>  CloudA;
typedef pcl::PointCloud<PointA>::Ptr  CloudAPtr;

struct Cluster{
    float x; 
    float y; 
    float z;
    float width;
    float height;
    float depth;
    float curvature;
    Vector3f min_p;
    Vector3f max_p;
};

struct Clusters{
    Cluster data;
    CloudA centroid;
    CloudA points;
};

void getClusterInfo(CloudA pt, Cluster& cluster)
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

void clustering(CloudAPtr cloud_in,
                vector<Clusters>& cluster_array){
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
    ec.setClusterTolerance (0.30); // 15cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (10000);
    ec.setSearchMethod (tree);
    ec.setInputCloud(cloud_in);
    ec.extract (cluster_indices);
    //reset z value
	for(int i=0;i<(int)cloud_in->points.size();i++)
        cloud_in->points[i].z=tmp_z[i];

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
        getClusterInfo(*cloud_cluster, data);

		PointA center;
		center.x = data.x;
		center.y = data.y;
		center.z = data.z;

        Clusters cluster;
        cluster.data = data;
        cluster.centroid.points.push_back(center);
        for(size_t i=0;i<cloud_cluster->points.size();i++)
            cluster.points.points.push_back(cloud_cluster->points[i]);

        cluster_array.push_back(cluster);
    }
}
