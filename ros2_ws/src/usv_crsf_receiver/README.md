# usv_crsf_receiver

Jetson Orin Nano 上的 ExpressLRS/CRSF ROS 2 串口驱动。该包只负责协议层，
暂不为任何通道赋予油门、转向或模式开关等业务含义。

## 话题

- `/rc/channels` (`usv_interfaces/msg/CrsfChannels`)：16 路原始值、微秒值、
  帧率、CRC 统计和连接状态。
- `/rc/link_statistics` (`usv_interfaces/msg/CrsfLinkStatistics`)：RSSI、LQ、
  SNR、天线、射频模式及发射功率代码。
- `/rc/connected` (`std_msgs/msg/Bool`)：超过 `frame_timeout` 未收到遥控帧时
  变为 `false`。

## 启动

```bash
cd /home/hai/usv_project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch usv_crsf_receiver crsf_receiver.launch.py
```

验证：

```bash
ros2 topic echo /rc/channels
ros2 topic echo /rc/link_statistics
ros2 topic echo /rc/connected
```

同一时刻只能有一个进程打开 `/dev/ttyTHS1`。启动 ROS 2 节点前，应先退出
`validation/crsf` 中的终端验证程序。
