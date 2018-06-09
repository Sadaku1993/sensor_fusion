#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>

#include <sensor_fusion/get_cluster_info.h>
#include <sensor_fusion/pub_cloud.h>
#include <sensor_fusion/kd_tree.h>

#define K 20

using namespace std;

ros::Publisher pub_referencePoints;
ros::Publisher pub_searchPoints;
ros::Publisher pub_candidatePoints;
ros::Publisher pub_centroidPoints;

void pc_Callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(*msg, *cloud);

    if(!cloud->empty())
    {
        cout << "\033[2J" << endl;
        
        Cluster cluster;
        getClusterInfo(*cloud, cluster);

        /*       z
                 |
              P1 | P2
            y----------
              P3 | P4
                 |
        */

        // Devide Cloud
        CloudAPtr cloud1(new CloudA);
        CloudAPtr cloud2(new CloudA);
        CloudAPtr cloud3(new CloudA);
        CloudAPtr cloud4(new CloudA);

        for(size_t i=0;i<cloud->points.size();i++)
        {
            if(cloud->points[i].y > cluster.y && cloud->points[i].z > cluster.z)
                cloud1->points.push_back(cloud->points[i]);
            else if(cluster.y > cloud->points[i].y && cloud->points[i].z > cluster.z)
                cloud2->points.push_back(cloud->points[i]);
            else if(cloud->points[i].y > cluster.y && cluster.z > cloud->points[i].z)
                cloud3->points.push_back(cloud->points[i]);
            else
                cloud4->points.push_back(cloud->points[i]);
        }

        // Reference Points
        PointA p0, p1, p2, p3, p4;
        p0.x = cluster.x; p0.y = cluster.y; p0.z = cluster.z; 
        p1.x = cluster.x; p1.y = cluster.y+0.15; p1.z = cluster.z+0.15;
        p2.x = cluster.x; p2.y = cluster.y-0.15; p2.z = cluster.z+0.15;
        p3.x = cluster.x; p3.y = cluster.y+0.15; p3.z = cluster.z-0.15;
        p4.x = cluster.x; p4.y = cluster.y-0.15; p4.z = cluster.z-0.15;

        // Candidate Points
        CloudAPtr searchPoints1(new CloudA);
        CloudAPtr searchPoints2(new CloudA);
        CloudAPtr searchPoints3(new CloudA);
        CloudAPtr searchPoints4(new CloudA);

        PointA searchPoint1, searchPoint2, searchPoint3, searchPoint4;
        PointA centroid1, centroid2, centroid3, centroid4;

        printf("Point1\n");
        neighbors_with_radius_Search(cloud1, p1, searchPoints1, searchPoint1);
        // K_nearest_neighbor_search(K, cloud1, searchPoint1, centroid1);
        K_nearest_neighbor_search(K, cloud1, p1, centroid1);

        printf("Point2\n");
        neighbors_with_radius_Search(cloud2, p2, searchPoints2, searchPoint2);
        // K_nearest_neighbor_search(K, cloud2, searchPoint2, centroid2);
        K_nearest_neighbor_search(K, cloud2, p2, centroid2);

        printf("Point3\n");
        neighbors_with_radius_Search(cloud3, p3, searchPoints3, searchPoint3);
        // K_nearest_neighbor_search(K, cloud3, searchPoint3, centroid3);
        K_nearest_neighbor_search(K, cloud3, p3, centroid3);

        printf("Point4\n");
        neighbors_with_radius_Search(cloud4, p4, searchPoints4, searchPoint4);
        // K_nearest_neighbor_search(K, cloud4, searchPoint4, centroid4);
        K_nearest_neighbor_search(K, cloud4, p4, centroid4);


        CloudAPtr referencePoints(new CloudA);
        referencePoints->points.push_back(p0);
        referencePoints->points.push_back(p1);
        referencePoints->points.push_back(p2);
        referencePoints->points.push_back(p3);
        referencePoints->points.push_back(p4);

        CloudAPtr searchPoints (new CloudA);
        *searchPoints += *searchPoints1;
        *searchPoints += *searchPoints2;
        *searchPoints += *searchPoints3;
        *searchPoints += *searchPoints4;

        CloudAPtr candidatePoints(new CloudA);
        candidatePoints->points.push_back(searchPoint1);
        candidatePoints->points.push_back(searchPoint2);
        candidatePoints->points.push_back(searchPoint3);
        candidatePoints->points.push_back(searchPoint4);

        CloudAPtr centroidPoints(new CloudA);
        centroidPoints->points.push_back(centroid1);
        centroidPoints->points.push_back(centroid2);
        centroidPoints->points.push_back(centroid3);
        centroidPoints->points.push_back(centroid4);


        // Publish 
        pub_cloud(referencePoints, msg->header, pub_referencePoints);
        pub_cloud(searchPoints,    msg->header, pub_searchPoints);
        pub_cloud(candidatePoints, msg->header, pub_candidatePoints);
        pub_cloud(centroidPoints,  msg->header, pub_centroidPoints);

    }
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "lidar_seg_circle");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cloud", 10, pc_Callback);
    pub_referencePoints = n.advertise<sensor_msgs::PointCloud2>("/output/reference", 10);
    pub_searchPoints = n.advertise<sensor_msgs::PointCloud2>("/output/search", 10);
    pub_candidatePoints = n.advertise<sensor_msgs::PointCloud2>("output/candidate", 10);
    pub_centroidPoints = n.advertise<sensor_msgs::PointCloud2>("/output/centroid", 10);
    ros::spin();

    return 0;
}
