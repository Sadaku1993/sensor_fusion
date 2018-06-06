## How to calibration lidar and camera
The following programs are prepared with reference to [this paper](https://arxiv.org/pdf/1705.04085.pdf).  
Source Code is [here](http://wiki.ros.org/velo2cam_calibration).

### Launch Sensor Node
```
$ roscd scripts/sensor_fusion
$ ./calibration.sh
```

### Turning a PointCloud into am Image
This Code is [pcl_ros](http://wiki.ros.org/pcl_ros/Tutorials/CloudToImage)
Let's assume you have an openni camera up and running. Fire up a terminal.
```
$rostopic list
/camera/depth/points2
/camera/rgb/camera_info
/camera/rgb/image_color
/camera/rgb/image_mono
```
To convert the point cloud to an image, just run the following
```
$rosrun pcl_ros convert_pointcloud_to_image input:=/camera/depth/points2 output:=/camera/depth/cloud_image
```
Then subscribe to the image and display it
```
rosrun image_view image_view image:=/camera/depth/cloud_image
```
