# Depth Image from SQ-LiDAR

## Bag Record
```
$roscd sensor_fusion/scripts/bagrec/
$./run_auto_savecloud.sh
```

## How to Make CNN-Depth Dataset

### Normal Estimation
```
$roscd sensor_fusion/launch/depthimage
$roslaunch sensor_fusion normal_estimation_local.launch
```

### Integrate PCD
```
$roscd sensor_fusion/launch/depthimage
$roslaunch sensor_fusion pcd_integrater.launch
```

### Create DepthImage
```
$roscd sensor_fusion/scripts/bagfile
$./depthimage.sh
```
