# USV全局位置预测（二维延迟卡尔曼滤波）

输入 `/usv/global_position`（UDP定位）与 `/imu/data_raw`。
输出 `/usv/position_prediction`（nav_msgs/Odometry）和
`/usv/position_prediction_valid`（std_msgs/Bool）。独立验证，不控制动力、不发布TF，
也不修改现有追踪估计器。

## 原理

状态 `[x,y,vx,vy,bax,bay]`：全局水平位置、速度、剩余水平加速度零偏。
使用IMU提供的四元数将原始加速度旋转到map并去除重力，再预测状态和协方差。
UDP位置按照 `header.stamp` 在历史状态上执行卡尔曼校正，再重放后续IMU和已接受
的位置测量，得到最新IMU时刻的预测。校正采用Joseph协方差更新和马氏距离门限。
这是借鉴延迟滤波逻辑的水面二维简化模型，不是论文完整三维姿态EKF的复现。

姿态直接采用IMU四元数，角速度使用IMU陀螺仪；本节点不重新估计姿态或陀螺零偏。
水平加速度零偏是map中的简化随机游走模型，不等价于完整三维IMU轴零偏模型。
Z固定为 `water_z`，忽略UDP的Z，不估计升沉。

## 必须满足的前提

- UDP坐标为同一map坐标系下的米制笛卡尔坐标，不是经纬度。
- UDP的测量时间与Jetson系统时间同步；测量时间不能换成网络接收时间。
- IMU姿态表示IMU到其参考世界系的旋转，参考世界系Z轴朝上。
- `imu_world_to_map_yaw_deg` 将IMU参考世界系对齐到UDP的map；需要实测，0不是已标定。
- 定位参考点必须与IMU参考点一致；天线/动捕标记存在杆臂时应先补偿，当前不补偿杆臂。
- `child_frame` 应与输入IMU的frame_id一致。实际IMU安装轴和船体轴不同，不可直接改名
  为base_link；需要上游正确的安装变换。

当前IMU磁航向可能受船上设备干扰，错误航向会把加速度旋转到错误方向。即使卡尔曼
滤波运行正常，也不保证全局预测准确。先静置和实际移动验证，不能仅凭100Hz输出
判断定位可靠。初始速度均值为0且具有较大不确定性，需要连续定位测量逐步收敛。

## 联合启动

每个终端先执行：

```bash
cd /home/hai/usv_project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

分别启动：

```bash
ros2 launch usv_imu_driver imu.launch.py
```

```bash
ros2 launch usv_udp_position udp_position.launch.py
```

```bash
ros2 launch usv_position_estimator position_estimator.launch.py
```

发送端须发送当前测量时间戳，不能使用示例中的旧Unix时间。
先启动IMU积累缓存，再开始发送位置；早于最初IMU缓存的位置会被拒绝。
首个定位直接初始化全局位置和零均值速度，不将很大的绝对坐标当作运动跳变；
早于该初始化定位时刻的乱序位置不再用于融合。

查看结果：

```bash
ros2 topic echo /usv/position_prediction
ros2 topic echo /usv/position_prediction_valid
ros2 topic hz /usv/position_prediction
```

Odometry位置在map，线速度在child_frame的IMU轴中表达，不要当作map速度读取。
输出时间戳是最新已处理IMU时间，不人为改成发布时刻。最多约100Hz，线程调度可能
略低；无定位初始化不发布位置，无新IMU不重复发布位置。

## 安全与调参

所有参数在 `config/position_estimator.yaml`，中文注释，修改后重启即可。

- `position_std_m` 按定位实际误差设定；过小容易过度相信异常定位。
- `acceleration_noise_std` 调大时协方差增长更快，更快接受定位校正。
- `history_seconds` 大于最大定位通信/处理延迟；缓存外的测量拒绝融合。
- 超前时间戳、不同坐标系、重复时间戳和异常创新不会正常进入融合。
- 定位超过 `position_timeout` 后valid为false，IMU正常时仍发布带增长协方差的预测。
- IMU超时停止位置发布；IMU大间隔/时钟倒退重置，并等待新定位初始化。
- 导航消费者必须同时检查valid和时间新鲜度；valid只表达时效，不代表完成航向标定。
- 不要把静置加速度Z直接当作线加速度；当前硬件平放Z约+9.8m/s²，因此需要去重力。

建议验证：静置时位置/速度稳定；沿map的X/Y轴分别移动检查方向；给相同数据增加
网络延迟检查估计是否一致；停止定位超过超时检查valid；停止IMU检查不再发布。

算法模拟测试：

```bash
PYTHONPATH=src/usv_position_estimator python3 -m pytest src/usv_position_estimator/test -q
```
