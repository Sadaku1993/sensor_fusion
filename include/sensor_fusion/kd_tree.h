#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA>  CloudA;
typedef pcl::PointCloud<PointA>::Ptr  CloudAPtr;

using namespace std;


void neighbors_with_radius_Search(CloudAPtr cloud,
                                  PointA point,
                                  CloudAPtr& searchPoints,
                                  PointA& centroid)
{
    printf("Reference Point ( %.3f %.3f %.3f )\n", point.x, point.y, point.z);
    
    // pointを中心に1000パーティクルをばらまく
    for(int i=0;i<1000;i++)
    {
        PointA searchPoint;
        searchPoint.x = point.x + 0.00005 * (rand()%1024 - 512);
        searchPoint.y = point.y + 0.00005 * (rand()%1024 - 512);
        searchPoint.z = point.z + 0.00005 * (rand()%1024 - 512);
        searchPoints->points.push_back(searchPoint);
    }

    // 近傍点群を探索し、cloudと重なる点群数が最も小さい中心点を算出
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquareDistance;
    float radius = 0.10;

    size_t SIZE = cloud->points.size();

    for(size_t i=0;i<searchPoints->points.size();i++)
    {
        PointA searchPoint = searchPoints->points[i];
        if(kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquareDistance) > 0){
            if( pointIdxRadiusSearch.size() < SIZE ){
                centroid = searchPoint;
                SIZE = pointIdxRadiusSearch.size();
            }
        }
    }
}

void K_nearest_neighbor_search(int K,
                               CloudAPtr cloud,
                               PointA searchPoint,
                               PointA& output)
{
    printf("K nearest neighbor search at ( %.3f %.3f %.3f with %d )\n", searchPoint.x, searchPoint.y, searchPoint.z, K);

    // Initialize
    float X = searchPoint.x;
    float Y = searchPoint.y;
    float Z = searchPoint.z;

    // 中心点から最近傍点群を指定サイズ取得し、重心点を円の重心点を算出する
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    
    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        {
            X += cloud->points[ pointIdxNKNSearch[i] ].x;
            Y += cloud->points[ pointIdxNKNSearch[i] ].y;
            Z += cloud->points[ pointIdxNKNSearch[i] ].z;
        }
        output.x = X / int(pointIdxNKNSearch.size() + 1);
        output.y = Y / int(pointIdxNKNSearch.size() + 1);
        output.z = Z / int(pointIdxNKNSearch.size() + 1);
    }

    printf("Final Centroid Point ( %.3f %.3f %.3f )\n", output.x, output.y , output.z);
}
