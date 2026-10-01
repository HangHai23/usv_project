# 无人船导航与小球追踪项目

项目将正式 ROS 2 源码、硬件验证程序、第三方原始资料和维护工具严格分开。
日常开发只需要关注 `ros2_ws/src`。

手动驾驶示范采集：`./start_manual_demonstration.sh left`（右球门使用 `right`）。
按S开始、E成功结束并分析；只记录，不改变遥控动力映射。
操作、安全说明和数据格式见 [手动示范采集说明](ros2_ws/src/usv_goal_navigation/README.md#手动示范采集与分析单向电调)。

## 目录导航

```text
usv_project/
├── ros2_ws/
│   └── src/                    # 正式 ROS 2 源码（唯一需要 colcon 构建的目录）
│       ├── usv_camera_driver/  # USB摄像头采集、自动发现与断线重连
│       ├── usv_yolo_detector/  # TensorRT Engine推理与小球检测框发布
│       ├── usv_ball_follow/    # 目标状态估计和IMU辅助追踪控制
│       ├── usv_imu_driver/     # IMU 节点及 RViz 配置
│       ├── usv_crsf_receiver/ # CRSF 接收机节点
│       ├── usv_rc_teleop/     # 遥控通道映射与示波器测试
│       ├── usv_pwm_actuator/  # 动力百分比到双路 PWM 输出
│       ├── usv_web_monitor/   # 局域网状态监控与运行参数调节
│       └── usv_interfaces/    # 项目自定义消息，不是运行节点
├── validation/                # 不依赖 ROS 2 的硬件验证程序
│   ├── imu/
│   ├── crsf/
│   └── vision/                # USB摄像头网页预览与YOLO图像采集
├── vendor/                    # 厂家/第三方原始源码，仅用于追溯和安装
│   ├── imu/
│   └── kernel/
├── docs/                      # 架构、硬件资料和厂家 PDF
│   └── reference/imu/
├── models/                    # ONNX、Jetson本机Engine和离线验证结果
└── tools/                     # 构建、清理和系统安装脚本
```

## 构建

```bash
/home/hai/usv_project/tools/build.sh
source /home/hai/usv_project/ros2_ws/install/setup.bash
```

`ros2_ws/build`、`install` 和 `log` 都是 colcon 自动生成的文件，不是源码，
不应编辑或提交。清理它们不会丢失项目代码：

```bash
/home/hai/usv_project/tools/clean_generated.sh
```

## 正式节点

USB摄像头（默认1280×720、30 Hz）：

```bash
ros2 launch usv_camera_driver camera.launch.py
```

YOLO TensorRT推理（Engine路径在YAML中配置）：

```bash
ros2 launch usv_yolo_detector yolo_detector.launch.py
```

目标估计与IMU辅助追踪控制：

```bash
ros2 launch usv_ball_follow ball_follow.launch.py
```

IMU（硬件与ROS默认100 Hz）：

```bash
ros2 launch usv_imu_driver imu.launch.py port:=/dev/ttyUSB0
```

CRSF 遥控接收机：

```bash
ros2 launch usv_crsf_receiver crsf_receiver.launch.py
```

PWM 执行器（默认 `/dev/i2c-7`、地址 `0x2D`）：

```bash
ros2 launch usv_pwm_actuator pwm_actuator.launch.py
```

CH3 控制 ESC1 的整体示波器测试：

```bash
ros2 launch usv_rc_teleop channel3_esc1_test.launch.py
```

当前正式遥控链使用 CH2 油门、CH4 转向、CH5 急停和 CH8 手动/自动模式选择；
自动导航命令发布节点尚未实现。包和话题的详细说明见
[docs/architecture.md](docs/architecture.md)。

局域网状态监控：

```bash
ros2 launch usv_web_monitor web_monitor.launch.py
```

独立 USB 摄像头与 YOLO 原始图像采集：

```bash
python3 validation/vision/yolo_usb_camera_capture.py
```

## 开发期配置

项目启动文件在检测到工作空间源码时，优先直接读取 `ros2_ws/src/*/config`
中的 YAML；没有源码的部署环境才读取 `install` 副本。因此修改当前工作空间
的 YAML 后只需重启节点，无需重新执行 `colcon build`。启动日志中的
`Live ... config:` 会显示本次实际读取的文件。
