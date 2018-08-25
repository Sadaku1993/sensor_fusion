#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace std;

class SAVER{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;

        CloudAPtr save_cloud;
        string file_path;
        int count;
        int save_count;
        int number;
        bool flag;

    public:
        SAVER();

        void Callback(const sensor_msgs::PointCloud2ConstPtr msg);
        void save_pointcloud(const sensor_msgs::PointCloud2ConstPtr msg);
        void savePCDFile(CloudAPtr cloud, 
                         int count);
        void reset();
};

SAVER::SAVER()
    : nh("~"),
      save_cloud(new CloudA)
{
    nh.getParam("save_count", save_count);
    nh.getParam("file_path" , file_path);
	sub = nh.subscribe("/cloud", 10, &SAVER::Callback, this);
    count = 0;
    number = 0;
    flag = false;
}

void SAVER::Callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    if(!flag){
        char text[50];
        printf("are you ready?(y/n):");
        scanf("%s", text);
        
        if(text[0]=='y'){
            flag = true;
            printf("\n");
        }
        else if(text[0]=='n'){
            flag = false;
            printf("\n");
        }
        else{
            flag = false;
            printf("prease input y/n\n");
        }
    }

    else{
        save_pointcloud(msg);
    }
}

void SAVER::save_pointcloud(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr input_cloud(new CloudA);
    CloudAPtr threshold_cloud(new CloudA);

    pcl::fromROSMsg(*msg, *input_cloud);
    for(size_t i=0;i<input_cloud->points.size();i++){
        double distance = sqrt(pow(input_cloud->points[i].x, 2)+ pow(input_cloud->points[i].y, 2) + pow(input_cloud->points[i].z, 2));
        if(2.0<distance && distance < 30)
            threshold_cloud->points.push_back(input_cloud->points[i]);
    }

    if(count < save_count){
		*save_cloud += *threshold_cloud;
		if(count % 100 == 0) printf("count:%d/%d Cloud_Size:%d\n", count, save_count, int(save_cloud->points.size()));
	}

	if(count == save_count){
		cout<<"Num:"<<number<<" Success Save PointCloud!!! Next Node"<<endl;
        savePCDFile(save_cloud, number);
        number++;
		reset();
	}
    
    count++;
}

void SAVER::savePCDFile(CloudAPtr cloud, 
                        int count)
{
    CloudAPtr save_cloud(new CloudA);
	pcl::copyPointCloud(*cloud, *save_cloud);
 
	save_cloud->width = 1;
	save_cloud->height = save_cloud->points.size();

    string file_name = to_string(count);
	pcl::io::savePCDFile(file_path+file_name+".pcd", *save_cloud);
    printf("Num:%d saved %d\n", count, int(cloud->points.size()));
}

void SAVER::reset()
{
    count = 0;
    flag = false;
    save_cloud->points.clear();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_saver");
    
    SAVER saver;

    ros::spin();

    return 0;
}
