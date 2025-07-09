# Dual Mid360 Data Combine

This ROS 2 package merges point clouds from two Livox Mid-360 LiDAR sensors with synchronized IMU data.

## Overview

This package provides a ROS 2 node that:
- Subscribes to two Livox Mid-360 LiDAR sensors
- Synchronizes point clouds from both sensors based on timestamps
- Applies coordinate transformations to merge the point clouds
- Publishes the merged point cloud and synchronized IMU data

## Prerequisites

- ROS 2 (tested with Humble)
- Livox ROS Driver 2
- PCL (Point Cloud Library)
- Eigen3

## Building

```bash
colcon build --packages-select merge_cloud
```

## Usage

1. Configure the transformation parameters in `config/merge_config.yaml`
2. Launch the node:
```bash
ros2 launch merge_cloud launch.py
```

## Configuration

The calibration parameters for both LiDAR sensors can be configured in `config/merge_config.yaml`:

```yaml
lidar1:
  roll: 1.5708       # 90 degrees
  pitch: -1.5708     # -90 degrees
  yaw: 1.5708        # 90 degrees
  tx: 0.11
  ty: 0.0
  tz: 0.0

lidar2:
  roll: 0.0
  pitch: -1.5708
  yaw: 0.0
  tx: 0.0
  ty: 0.0
  tz: 0.0
```

## Topics

### Subscribed Topics
- `/livox/lidar_192_168_1_151` (livox_ros_driver2/msg/CustomMsg) - First LiDAR sensor
- `/livox/lidar_192_168_1_3` (livox_ros_driver2/msg/CustomMsg) - Second LiDAR sensor
- `/livox/imu_192_168_1_151` (sensor_msgs/msg/Imu) - IMU data

### Published Topics
- `/merged_cloud` (livox_ros_driver2/msg/CustomMsg) - Merged point cloud
- `/cloud_registered_body/imu` (sensor_msgs/msg/Imu) - Transformed IMU data

## License

TODO: Add license information
