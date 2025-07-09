# 双Mid360数据融合

这个ROS 2包用于合并两个Livox Mid-360激光雷达传感器的点云数据，并同步IMU数据。

## 概述

这个包提供了一个ROS 2节点，功能包括：
- 订阅两个Livox Mid-360激光雷达传感器的数据
- 基于时间戳同步两个传感器的点云数据
- 应用坐标变换来合并点云
- 发布合并后的点云和同步的IMU数据

## 依赖项

- ROS 2 (已在Humble版本上测试)
- Livox ROS Driver 2
- PCL (点云库)
- Eigen3

## 构建

```bash
colcon build --packages-select merge_cloud
```

## 使用方法

1. 在`config/merge_config.yaml`中配置变换参数
2. 启动节点：
```bash
ros2 launch merge_cloud launch.py
```

## 配置

可以在`config/merge_config.yaml`中配置两个激光雷达传感器的标定参数：

```yaml
lidar1:
  roll: 1.5708       # 90度
  pitch: -1.5708     # -90度
  yaw: 1.5708        # 90度
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

## 话题

### 订阅话题
- `/livox/lidar_192_168_1_151` (livox_ros_driver2/msg/CustomMsg) - 第一个激光雷达传感器
- `/livox/lidar_192_168_1_3` (livox_ros_driver2/msg/CustomMsg) - 第二个激光雷达传感器
- `/livox/imu_192_168_1_151` (sensor_msgs/msg/Imu) - IMU数据

### 发布话题
- `/merged_cloud` (livox_ros_driver2/msg/CustomMsg) - 合并后的点云
- `/cloud_registered_body/imu` (sensor_msgs/msg/Imu) - 变换后的IMU数据

## 许可证

待添加许可证信息
