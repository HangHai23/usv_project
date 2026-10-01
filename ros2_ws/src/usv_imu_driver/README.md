# usv_imu_driver

Yahboom 串口 IMU 的正式 ROS 2 驱动。厂家原始资料和未修改包位于项目根目录
`vendor/imu`，独立串口验证程序位于 `validation/imu`。

## 话题

- `/imu/data_raw` (`sensor_msgs/Imu`)
- `/imu/mag` (`sensor_msgs/MagneticField`)
- `/imu/marker` (`visualization_msgs/Marker`)
- `/baro`、`/euler` (`std_msgs/Float32MultiArray`)

## 启动

仅启动驱动：

```bash
ros2 launch usv_imu_driver imu.launch.py port:=/dev/ttyUSB0
```

启动厂家 IMU/Mag RViz 插件界面：

```bash
ros2 launch usv_imu_driver imu_display.launch.py port:=/dev/ttyUSB0
```

启动 TF + Marker 简化验证界面：

```bash
ros2 launch usv_imu_driver imu_marker_display.launch.py port:=/dev/ttyUSB0
```
