# USV Web Monitor

## 视觉追踪模块

网页订阅并显示：

- `/ball/debug_image`：YOLO标注画面，以MJPEG传输到浏览器。
- `/ball/detection`：类别、置信度、分辨率、框中心和推理耗时。
- `/ball/target_state`：估计方位角、追踪/预测/丢失状态和IMU补偿。
- `/ball/desired_yaw_rate`：控制器期望偏航角速度。
- `/ball/follow_active`：自动追踪控制是否有效。

`usv_yolo_detector/config/yolo_detector.yaml`中的`publish_debug_image`必须为
`true`。没有摄像头或推理节点时，视觉模块只显示“等待推理”，不会使用演示数据。

一个 ROS 2 局域网监控面板，展示遥测数据，并允许受限地调整运行时动力参数：

- 全局动力限幅（0～100%）
- 左右电机微调（-100～100%）

网页通过 ROS 2 原子参数服务修改当前 `/pwm_actuator` 进程，不写入 YAML。
节点重启后仍以 `src/usv_pwm_actuator/config/pwm_actuator.yaml` 为准。

- CRSF/ExpressLRS RSSI、链路质量、SNR 和帧统计；
- CH8 控制模式仲裁结果与最终动力来源；
- 双路请求动力、实际动力和 PWM 脉宽；
- CH5 紧急停止、命令看门狗和 dry-run 状态。
- Jetson 内存容量与使用率、CPU 使用率和运行时间；
- CPU/TJ/SOC 温度；
- INA3221 `VDD_IN`（或最接近的输入轨）供电电压。

## 安装位置

将整个 `usv_web_monitor` 文件夹复制到 Jetson：

```text
~/usv_project/ros2_ws/src/usv_web_monitor
```

## 安装依赖并构建

```bash
sudo apt update
sudo apt install -y python3-aiohttp

cd ~/usv_project/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select usv_web_monitor
source install/setup.bash
```

## 启动

先正常启动无人船控制链，再打开另一个终端：

```bash
source /opt/ros/humble/setup.bash
source ~/usv_project/ros2_ws/install/setup.bash
ros2 launch usv_web_monitor web_monitor.launch.py
```

服务默认监听全部网卡的 `8080` 端口。查询 Jetson 地址：

```bash
hostname -I
```

同一局域网内的手机、平板或电脑打开：

```text
http://JETSON_IP:8080
```

例如 Jetson 地址为 `192.168.1.80`：

```text
http://192.168.1.80:8080
```

可修改端口：

```bash
ros2 launch usv_web_monitor web_monitor.launch.py port:=8090
```

## 检查服务

```bash
curl http://127.0.0.1:8080/healthz
ss -lntp | grep 8080
```

页面只订阅 ROS 2 话题，不提供控制接口，不会向无人船发送指令。

未连接 WebSocket 时，页面会明确显示“演示数据”。也可以强制预览：

```text
http://JETSON_IP:8080/?demo=1
```

## 可选：设置开机启动

确认手动启动正常后执行：

```bash
sudo cp ~/usv_project/ros2_ws/src/usv_web_monitor/systemd/usv-web-monitor.service \
  /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now usv-web-monitor
systemctl status usv-web-monitor --no-pager
```

查看运行日志：

```bash
journalctl -u usv-web-monitor -f
```

## 电压数据检查

网页会扫描 `/sys/class/hwmon` 中名称包含 `ina` 的监控器，并优先选择
`VDD_IN`、`VIN_SYS_5V0`、`VDDIN` 或 `VIN`。若网页显示“未发现 VDD_IN”，运行：

```bash
for h in /sys/class/hwmon/hwmon*; do
  echo "=== $h : $(cat "$h/name" 2>/dev/null) ==="
  grep -H . "$h"/in*_label "$h"/in*_input 2>/dev/null
done
```

将输出保存下来即可据此适配具体 Jetson 载板的电源轨名称。
