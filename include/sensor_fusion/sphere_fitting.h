#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

Eigen::Vector4f sphere_fitting(CloudA cloud)
{
    Eigen::Matrix4f matrix;
    matrix = Eigen::Matrix4f::Zero();

    Eigen::Vector4f v;
    v = Eigen::Vector4f::Zero();

    for(size_t i=0;i<cloud.points.size();i++){
        matrix(0,0) += 2.0*cloud[i].x*cloud[i].x;
        matrix(0,1) += 2.0*cloud[i].x*cloud[i].y;
        matrix(0,2) += 2.0*cloud[i].x*cloud[i].z;
        matrix(0,3) += 2.0*cloud[i].x;

        matrix(1,0) += 2.0*cloud[i].y*cloud[i].x;
        matrix(1,1) += 2.0*cloud[i].y*cloud[i].y;
        matrix(1,2) += 2.0*cloud[i].y*cloud[i].z;
        matrix(1,3) += 2.0*cloud[i].y;

        matrix(2,0) += 2.0*cloud[i].z*cloud[i].x;
        matrix(2,1) += 2.0*cloud[i].z*cloud[i].y;
        matrix(2,2) += 2.0*cloud[i].z*cloud[i].z;
        matrix(2,3) += 2.0*cloud[i].z;

        matrix(3,0) += 2.0*cloud[i].x;
        matrix(3,1) += 2.0*cloud[i].y;
        matrix(3,2) += 2.0*cloud[i].z;
        matrix(3,3) += 2.0 * 1.0;

        v[0] += cloud[i].x * (cloud[i].x*cloud[i].x+cloud[i].y*cloud[i].y+cloud[i].z*cloud[i].z);
        v[1] += cloud[i].y * (cloud[i].x*cloud[i].x+cloud[i].y*cloud[i].y+cloud[i].z*cloud[i].z);
        v[2] += cloud[i].z * (cloud[i].x*cloud[i].x+cloud[i].y*cloud[i].y+cloud[i].z*cloud[i].z);
        v[3] += cloud[i].x*cloud[i].x+cloud[i].y*cloud[i].y+cloud[i].z*cloud[i].z;
    }
        
    Eigen::Vector4f vector = matrix.inverse()*v;

    // cout << "matrix" << endl <<matrix << endl;
    // cout << "v" <<  endl << v <<endl;
    // cout <<"vector" << endl << vector << endl;

    float range = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2] + 2*vector[3]);
    
    printf("x:%.2f y:%.2f z:%.2f r:%.2f\n", vector[0], vector[1], vector[1], range);

    return vector;
}
