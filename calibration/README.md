## How to calibration lidar and camera
The following programs are prepared with reference to [this paper](https://arxiv.org/pdf/1705.04085.pdf).  
Source Code is [here](http://wiki.ros.org/velo2cam_calibration).

## On Board

### Launch Sensor Node
```
$roscd sensor_fusion/scripts/sq2
$./sensor_node.sh
```

### Launch Calibration Node
```
$roscd sensor_fusion/scropts/sq2
$./calibration.sh
```

## Bag Record
```
$roscd sensor_fusion/scripts/bagrec
$./run_calibration.sh
```

## Bagfiles

### DownLoad BagFiles
comming soon...

### Launch Calibration Node and rosbag play
```
$roscd sensor_fusion/scripts/bagfile
$./calibration.sh
```
