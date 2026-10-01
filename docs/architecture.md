# 当前软件架构

## 数据边界

```text
USB Camera ──V4L2──> usv_camera_driver
                         └── /camera/image_raw ──> usv_yolo_detector
                                                        ├── /ball/detection
                                                        └── /ball/debug_image (optional)

/ball/detection + /imu/data_raw ──> target_state_estimator
                                           └── /ball/target_state

/ball/target_state + /imu/data_raw ──> ball_follow_controller
                                             └── /propulsion/automatic_command

Yahboom IMU ──USB/Serial──> usv_imu_driver
                              ├── /imu/data_raw
                              ├── /imu/mag
                              ├── /euler
                              └── /baro

ELRS Receiver ──CRSF/UART──> usv_crsf_receiver
                              ├── /rc/channels
                              ├── /rc/link_statistics
                              └── /rc/connected

                    usv_interfaces
                          └── CrsfChannels / CrsfLinkStatistics

RC mapper / Navigation / Safety arbiter
                  └── /propulsion/command
                            │
                            v
manual mixer ──/propulsion/manual_command──┐
                                           ├─ control_mode_arbiter
navigation ─/propulsion/automatic_command─┘          │
                                             /propulsion/command
                                                     │
                    usv_pwm_actuator ──I2C-7 / address 0x2D──> PWM board
                            └── /actuators/pwm_state
```

IMU节点每次启动会主动把硬件上报率设置为100 Hz，并以100 Hz发布姿态、原始惯性
数据和磁场数据，避免设备断电后退回约25 Hz而ROS仍重复发布缓存。当前气压计没有
实际串口上报帧，`/baro`不能作为有效传感器数据使用。

驱动层只负责读取、校验、单位转换和发布。遥控通道的油门/转向含义、失控保护、
船体控制、状态估计、导航和视觉追踪应分别放在后续独立包中。

`usv_pwm_actuator` 只接受经过上层仲裁的最终两路有符号动力百分比。遥控器和
自动导航不应同时直接发布到该最终话题；后续应由独立 command mux/safety
节点选择唯一来源。PWM 节点本身提供启动回中、退出回中和命令超时回中。

当前 `usv_rc_teleop/channel3_esc1_mapper` 是示波器单通道测试映射：CH3
`1000..1500..2000 us` 映射为 ESC1 `-100..0..100%` 请求，ESC2固定为0%。
它用于验证完整脉宽链路，不是最终的双电机操纵或遥控/导航仲裁实现。

CRSF 使用 `/dev/ttyTHS1 @ 420000`。PWM 板默认使用 `/dev/i2c-7`、地址
`0x2D`，从而避免 UART 波特率冲突；若改用 UART 后端，必须使用独立 UART。

CH8低位选择手动候选命令，高位选择自动候选命令。遥控链路断开、所选命令
超时或自动节点尚未启动时，仲裁器只发布中位命令。PWM执行层还直接监视RC
链路和CH5急停，因此上层仲裁故障不能绕过最终回中保护。

## 为什么 install 中有很多文件

`usv_interfaces` 的 `.msg` 会由 ROS 2 自动生成 C、C++、Python 以及 DDS 类型
支持库，所以 `install` 的文件数远大于手写源码。这些不是下载的第三方包，
也不是重复源码。开发者只应阅读 `ros2_ws/src`，需要时可删除全部生成目录并
从源码重建。

## 第三方边界

- `vendor/imu/YbImuLib`：厂家 Python 库原件。
- `vendor/imu/imu_ros2_device_original`：厂家 ROS 2 示例原件，仅供对照。
- `vendor/kernel/CH341SER`：Jetson CH340/CH341 内核模块源码。
- `docs/reference/imu`：厂家操作 PDF。
