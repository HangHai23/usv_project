# IMU辅助小球追踪控制

本包包含两个节点：

- `target_state_estimator`：检测框转方位角、alpha-beta滤波、IMU帧间预测和丢帧状态。
- `ball_follow_controller`：目标方位外环、IMU偏航角速度内环、油门调度和双电机混控。

```text
/ball/detection + /imu/data_raw
              -> /ball/target_state
              -> /propulsion/automatic_command
```

启动：

```bash
ros2 launch usv_ball_follow ball_follow.launch.py
```

调试：

```bash
ros2 topic echo /ball/target_state
ros2 topic echo /ball/desired_yaw_rate
ros2 topic echo /ball/follow_active
ros2 topic echo /propulsion/automatic_command
```

配置文件直接从源码读取，修改后重启即可：

- `config/target_estimator.yaml`
- `config/ball_follow_controller.yaml`

实船前必须低功率确认两个方向：目标位于画面右侧时动力应使船右转；船正在右转且目标已经居中时，控制器应给出左转方向的阻尼。方向错误时只修改YAML中的
`camera_horizontal_reversed`、`imu_yaw_rate_reversed`或`steering_reversed`。
