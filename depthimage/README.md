# Depth Image from SQ-LiDAR

## Bag Record
```
$roscd sensor_fusion/scripts/bagrec/
$./run_auto_savecloud.sh
```

## How to Make CNN-Depth Dataset
```
$roscd sensor_fusion/launch/depthimage
```

### Normal Estimation
```
$roslaunch sensor_fusion normal_estimation_local.launch
```

### Min-Max
```
$roslaunch sensor_fusion min_max.launch
```

### Integrate PCD
```
$roslaunch sensor_fusion pcd_integrater_ground.launch
$roslaunch sensor_fusion pcd_integrater_obstacle.launch
$roslaunch sensor_fusion pcd_integrater.launch
```

### Create DepthImage
```
$roscd sensor_fusion/scripts/bagfile
$./depthimage.sh
```

### Bagfile and PCD
bagfiles/sq2/SII/depthimage/nignh_v1.bag --- PCD/SQ2/20180717
bagfiles/sq2/SII/depthimage/perfect_night.bag --- PCD/SQ2/SII/perfect_night
