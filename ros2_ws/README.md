# ROS 2 工作空间

当前临时硬件为单向电调：正式PWM配置`forward_only: true`，停机1000us、满前进2000us，
负动力截为0，仍保留全局限幅。本文后续旧双向1500us停机说明仅适用于恢复双向配置时。
启动阈值测试见
[单向电调启动阈值项目](../validation/esc_start_threshold/README.md)。

`src` 是源码目录。`build`、`install`、`log` 由 colcon 生成，可随时通过项目
根目录的 `tools/clean_generated.sh` 删除，再运行 `tools/build.sh` 重建。

| 包 | 类型 | 职责 |
|---|---|---|
| `usv_imu_driver` | Python 节点 | 串口 IMU、磁场、姿态和 RViz |
| `usv_crsf_receiver` | Python 节点 | CRSF 串口解析与链路状态 |
| `usv_rc_teleop` | Python 节点 | 遥控通道到动力指令的映射与测试 |
| `usv_pwm_actuator` | Python 节点 | 最终动力百分比转换为双路 PWM 控制命令 |
| `usv_interfaces` | 消息接口 | 无人船自定义消息 |

另外还有以下正式软件包：

| 包 | 职责 |
|---|---|
| `usv_camera_driver` | 自动选择 USB 摄像头并发布 1280×720 图像 |
| `usv_yolo_detector` | 使用 TensorRT engine 检测小球并发布检测框 |
| `usv_ball_follow` | 融合检测框与 IMU，估计目标状态并计算自动动力 |
| `usv_web_monitor` | 显示视觉、控制、安全和 Jetson 系统状态 |
| `usv_udp_position` | 接收局域网UDP全局位置，发布测量和接收时间戳 |
| `usv_position_estimator` | IMU预测、延迟UDP位置校正与历史重传播，输出全局二维位置/速度 |
| `usv_goal_navigation` | 地图记忆、延迟视觉重放+IMU/动力模型连续预测、左右球门路线跟踪；视觉中断10秒停机；完整航行日志与离线增益/延迟分析，详见包内README |

全局位置预测的坐标对齐、时间同步及联合验证方法见
[位置预测节点说明](src/usv_position_estimator/README.md)。未接入自动动力控制。

UDP定位节点独立启动，协议、配置和使用方法见
[usv_udp_position说明](src/usv_udp_position/README.md)。当前使用米制全局笛卡尔
坐标，不直接将经纬度用于控制，也未加入追踪一键脚本。

## 一键启动

### 正式追踪

正式模式需要遥控器在线，会真实访问 PWM 控制板并输出信号：

```bash
cd /home/hai/usv_project
./start_tracking.sh
```

- CH2：油门。
- CH4：左右转向。
- CH5 高位：底层急停，两个输出立即回到 1500 us。
- CH8 低位：手动模式；CH8 高位：自动追踪模式。
- 遥控器断联：底层切断动力。

### 无遥控器台架模式

```bash
cd /home/hai/usv_project
./start_bench_tracking.sh
```

台架模式强制选择自动控制，不启动 CRSF 接收机，临时禁用依赖 RC 的急停检查，
并启用 `dry_run`。系统仍完成左右动力、限幅和 PWM 脉宽计算，但不会访问 PWM
控制板。两个脚本都会启动 IMU、摄像头、YOLO、目标估计、自动控制、网页和 CSV
记录。按 `Ctrl+C` 可停止本次脚本启动的所有进程。

网页地址为 `http://<Jetson-IP>:8080`，通过以下命令查看地址：

```bash
hostname -I
```

更换网页端口：

```bash
WEB_PORT=8081 ./start_bench_tracking.sh
```

CSV 默认保存到 `/home/hai/Downloads/usv_logs/`。

## 数据链路与联合调试顺序

主要数据流：

```text
摄像头 → YOLO检测 → 目标状态估计 ─┐
                    ↑ IMU          ├→ 自动跟踪控制器
遥控器 → 手动差速动力 ─────────────┤
                                   ↓
                         手动/自动模式仲裁
                                   ↓
                         限幅、微调与PWM转换
                                   ↓
                              左右电调
```

不要一开始同时修改所有增益。应从上游到下游逐层确认，否则方向、滤波、仲裁和
执行器问题会互相掩盖。

### 1. 检查传感器和视觉

```bash
ros2 topic hz /imu/data_raw
ros2 topic echo /ball/detection
ros2 topic echo /ball/target_state
```

正常情况下 IMU 接近 100 Hz；识别到球时 `detected=true`、
`target_valid=true`。目标在画面右侧时 `center_x>0.5` 且 `bearing_rad>0`，
左侧时角度为负。若图像角度反向，修改 `camera_horizontal_reversed`；若内部 IMU
角速度方向反向，修改 `imu_yaw_rate_reversed`。不要先用控制增益掩盖方向错误。

### 2. 检查自动控制器

```bash
ros2 topic echo /ball/desired_yaw_rate
ros2 topic echo /propulsion/automatic_command
```

目标在左右两侧时应产生相反的差速动力。若自动控制的左右关系完全相反，修改
`ball_follow_controller.yaml` 中的 `steering_reversed`。

### 3. 检查模式仲裁

```bash
ros2 topic echo /control/automatic_mode
ros2 topic echo /propulsion/command
```

正式自动模式应看到 `automatic_mode=true`，且最终指令的 `source` 包含
`automatic:ball_follow`。`automatic_command` 有值但最终 `command` 为零，通常是
CH8 未切到自动、RC断联、自动指令超时，或者没有使用台架旁路启动。

### 4. 检查执行器结果

```bash
ros2 topic echo /actuators/pwm_state
```

- `requested_percent`：仲裁器送入执行器的动力。
- `applied_percent`：经过全局限幅和通道微调后的动力。
- `pulse_width_us`：经过启动脉宽补偿后的最终 PWM。
- `watchdog_active`：动力指令是否超时。
- `emergency_stop_active`：CH5、RC断联或底层急停是否生效。
- `dry_run`：是否只计算而不写入硬件。

## 自动跟踪联合调参

控制器配置位于 `src/usv_ball_follow/config/ball_follow_controller.yaml`，目标估计
配置位于 `src/usv_ball_follow/config/target_estimator.yaml`。文件内每个参数均有
中文注释。

| 现象 | 优先调整 | 调整逻辑 |
|---|---|---|
| 小角度偏差没有转向 | `bearing_deadband_rad` | 减小，使更小偏差进入控制 |
| 小角度转向太弱 | `bearing_kp` | 增大，相同偏角产生更高期望角速度 |
| 大角度转向不明显 | `maximum_yaw_rate_rad_s` | 增大期望转向速度上限 |
| 左右动力差不够 | `maximum_steering_percent` | 增大最大差速动力 |
| 实际角速度低、纠偏不够强 | `yaw_rate_kp` | 增大角速度误差产生的转向动力 |
| 动力建立太慢 | `output_slew_rate_percent_s` | 增大动力变化斜率 |
| 目标居中后仍继续转、过冲 | `maximum_yaw_rate_rad_s`、`yaw_rate_kp` | 先降低最大角速度，再适当增强制动反馈 |
| 目标左右来回振荡 | `bearing_kp`、`yaw_rate_kp` | 逐步减小，必要时增大死区 |
| 追踪慢但没有振荡 | `bearing_kp`、`maximum_yaw_rate_rad_s` | 小步增大并观察是否开始过冲 |
| 转向时前冲过多 | `steering_slowdown_angle_rad` | 减小，使较小偏角就开始降低前进动力 |
| 球已很近仍前进 | `stop_area_ratio` | 减小，使较小框面积就停船 |
| 丢帧时前冲过多 | `predicting_throttle_scale` | 减小预测阶段的前进动力 |
| 检测框抖动导致角度抖动 | `filter_alpha`、`filter_beta` | 适当减小以增强平滑，但响应会变慢 |
| 快速目标估计跟不上 | `filter_alpha`、`filter_beta` | 适当增大，使估计更快跟随检测 |
| 正常快速变化被拒绝 | `outlier_gate_rad` | 适当增大检测离群门限 |
| 错误检测频繁进入控制 | `minimum_confidence` | 增大最低置信度 |

### 推荐调参步骤

1. 使用台架模式确认目标角度、IMU和左右动力方向正确。
2. 保持低巡航动力，先调 `bearing_kp` 和 `maximum_yaw_rate_rad_s`。
3. 再调 `yaw_rate_kp`，让实际角速度跟随期望角速度。
4. 出现过冲时先限制最大角速度，再处理内环增益和动力斜率。
5. 最后调整前进动力、接近停船距离和短时丢帧行为。
6. 每次只改一至两个相关参数，并用 CSV 对比修改前后的结果。

调试时重点观察这些 CSV 列：

```text
bearing_deg
desired_yaw_rate_rad_s
target_yaw_rate_rad_s
automatic_left_percent
automatic_right_percent
final_left_percent
final_right_percent
pwm_left_us
pwm_right_us
```

## 短时丢失目标

目标状态分为 `0 LOST`、`1 TRACKING` 和 `2 PREDICTING`。短时没有检测时，估计器
利用 IMU 和历史目标速度继续预测；超过 `lost_timeout` 后判定彻底丢失并停止动力。

- 水面反光造成几帧中断：可适当增加 `prediction_timeout`。
- 目标消失后船仍运动太久：减小 `lost_timeout` 和
  `predicting_throttle_scale`。
- 安全优先：把 `predicting_throttle_scale` 设为 `0.0`，预测时停止前进。

## 电机与 PWM 调整

执行器配置位于 `src/usv_pwm_actuator/config/pwm_actuator.yaml`。当前约定 PWM
通道1为左电机，通道2为右电机。

### 全局动力限幅

`output_limit_percent` 是最终全局限幅。例如设为 `50.0` 后，算法的
`-100~100%` 会线性映射到实际 `-50~50%`。网页修改的是当前运行节点；重启会
重新读取 YAML，因此需要长期保留的值必须写回 YAML。

### 左右动力微调

若相同指令下某侧稳态推力偏强，给该侧负微调，例如：

```yaml
channel_1_trim_percent: -5.0
channel_2_trim_percent: 0.0
```

不要用微调解决某侧较晚启动，启动问题应使用独立启动脉宽补偿。

### 启动脉宽补偿

若实测左电机需要 1560 us 才能稳定正向启动，右电机需要 1520 us：

```yaml
channel_1_forward_start_us: 1560
channel_2_forward_start_us: 1520
```

反向同理调整 `channel_1_reverse_start_us` 和 `channel_2_reverse_start_us`。
设置为 `1500` 表示禁用该方向补偿。

## 常见问题

### `AMENT_TRACE_SETUP_FILES: 未绑定的变量`

原因是在加载 ROS 环境前启用了 `set -u`。项目一键脚本已修复加载顺序；自行写
脚本时应先 `source` ROS 环境，再按需启用 `set -u`。

### 修改 YAML 后仍显示旧值

确认修改的是 `ros2_ws/src/<package>/config/*.yaml`，保存后停止旧节点并重新启动。
本项目 launch 优先读取源码目录 YAML。只改 YAML 不需要构建；修改 Python、
launch、`setup.py` 或消息接口后需要构建。

查询当前真实参数：

```bash
ros2 param get /pwm_actuator output_limit_percent
ros2 param get /ball_follow_controller bearing_kp
```

### 网页端口 8080 被占用

```bash
sudo ss -ltnp | grep ':8080'
```

若网页节点已经运行，直接访问即可；否则通过 `WEB_PORT=8081` 更换端口。

### 网页显示未连接

网页依据真实 ROS 数据流判断连接，而不是只判断进程是否存在：

```bash
ros2 topic hz /rc/channels
ros2 topic hz /imu/data_raw
ros2 topic hz /ball/detection
```

还应确认各终端使用相同的 `ROS_DOMAIN_ID`。

### IMU串口不存在或权限不足

```bash
ls -l /dev/myimu /dev/ttyUSB0
groups
sudo usermod -aG dialout hai
```

加入 `dialout` 后需要重新登录。不要把 `chmod 777` 作为长期方案，设备重插或重启
后权限会恢复。通过 `ros2 topic hz /imu/data_raw` 验证频率，当前硬件已实测可稳定
达到约100 Hz。

### 摄像头没有画面

```bash
ls -l /dev/video*
ros2 topic hz /camera/image_raw
```

默认 `device: auto` 会遍历设备。多个视频节点时可在 `camera.yaml` 固定正确的
`/dev/videoX`。

### YOLO没有检测结果

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /ball/detection
ls -lh /home/hai/usv_project/models/best.engine
```

确认 engine 在当前 Jetson/TensorRT 环境生成、类别 ID 正确且置信度门限合理。
TensorRT engine 通常不能直接复制到不同 GPU 平台使用。

### 自动动力有值但 PWM 不变化

```bash
ros2 topic echo /propulsion/automatic_command
ros2 topic echo /propulsion/command
ros2 topic echo /actuators/pwm_state
```

- 第一层有值、第二层为零：检查模式仲裁、CH8或RC连接。
- 第二层有值、`applied_percent` 为零：检查急停、看门狗和限幅。
- `pulse_width_us` 变化但示波器不变：检查 `dry_run`、I²C总线、地址和接线。

### 方向相反

按层排查，一次只改一个反向参数：

1. 图像角度反向：`camera_horizontal_reversed`。
2. IMU角速度反向：跟踪配置中的 `imu_yaw_rate_reversed`。
3. 自动差速反向：`steering_reversed`。
4. 单个电机机械方向反向：`left_motor_reversed` 或
   `right_motor_reversed`。

## 构建与清理

完整构建：

```bash
cd /home/hai/usv_project
./tools/build.sh
```

只构建指定包：

```bash
cd /home/hai/usv_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select usv_ball_follow --symlink-install
```

清理生成目录并重建：

```bash
cd /home/hai/usv_project
./tools/clean_generated.sh
./tools/build.sh
```

不要修改 `build`、`install` 或 `log` 中的副本；正式修改均应放在 `src` 下。
