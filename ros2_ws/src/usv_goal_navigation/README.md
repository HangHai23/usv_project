# 球门导航 v2：连续预测、地图记忆和实船标定日志

功能：CH8低位仍为手动，切到高位后从当时的预测位置建立到指定球门中心的直线路线。
返回低位结束本航次，遥控接管；再切高位重新建立起点。保留CH5急停与遥控失联停机。
单向电调只输出前进，左1右2。正偏航为逆时针，对应右电机加速；
2026-10-01全局场地日志确认原始IMU与视觉航向同号，设置imu_yaw_rate_reversed=false，不能直接沿用图像追踪的反向设置。

## 启动

```bash
cd /home/hai/usv_project
./start_goal_navigation.sh left
# 或选择右球门
./start_goal_navigation.sh right
```

脚本包含IMU、UDP接收、导航、遥控/仲裁/PWM、网页，不需要启动相机/YOLO或追球控制器。
启动前停止旧联合程序，避免重复打开串口/UDP/网页端口或重复发布自动动力。
CH8低位先验证遥控与CH5急停，查看状态后再切高位。

```bash
cd /home/hai/usv_project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo --full-length /navigation/status
```

只运行算法、不接管动力（先单独启动IMU与UDP接收）：

```bash
ros2 launch usv_goal_navigation goal_navigation.launch.py armed:=false require_mode:=false goal_side:=left
```

正式脚本显式设置armed=true、require_mode=true、自动仲裁输入话题；其余参数从src配置读取。
修改 `config/goal_navigation.yaml` 后重启生效。launch参数未显式指定时沿用YAML，不再以固定默认值覆盖。

## 地图与连续预测

- 首次收到有效场地宽高、球门端点就记忆；后续遗漏不清空。同一coord内保持首个有效值。
- 球门两侧可以分不同帧补齐。即使该帧没有本船，也能保存场地数据。
- 边界按协议size_m还原为 `(0,0),(W,0),(W,H),(0,H)` 的矩形近似，非实际弯曲边界。
- 地图缓存：`/home/hai/usv_project/data/navigation/map_cache.json`。
- 重启后仅收到相同coord的有效新报文才复用磁盘地图；不会只靠旧文件启动航行。
- Mac重启或重标定导致session/coord改变时停止，重启UDP及导航节点以重新建立地图。
- 连续预测：IMU积分偏航；实际PWM相对独立启动阈值的增量驱动一阶前进速度模型。
- PWM归零后模拟惯性减速，预测位置不会瞬间冻结。不直接将含重力/波浪的加速度两次积分成位置。
- 视觉以“Jetson本地接收时刻−拟合延迟0.2秒”放入历史轨迹，修正历史状态后重放至当前。
- 坏包、空观测、重复/乱序帧不清空预测，不刷新新位置计时。
- **按本次用户要求：新位置中断超过10秒停机。** IMU故障、PWM反馈超时、急停、模式失效仍会停机。
- `uncertainty_estimate_m`是启发式误差尺度，不是统计保证或卡尔曼协方差。
- 位置创新量过大时拒绝更新并记日志；若持续拒绝，检查坐标/船ID/航向偏置后重新启动。

这是无障碍场地的直线前视跟踪。目标为门线中心，进入到达半径后停止请求动力；
不包含穿门、球门柱避碰、复杂边界避障。边界当前缓存并用于越界诊断。
位置中断期间到达判断依赖预测，不能当作真实到达的独立测量。

## 占位模型与动力映射

以下为20261001_160703实船日志拟合后采用的候选值，仍需下一轮验证：

| 参数 | 初值 | 含义 |
|---|---:|---|
| speed_gain_m_s_per_us | 0.00415 | 每1us平均有效PWM的稳态前进速度，m/s |
| turn_gain_rad_s_per_us | 0.00335 | 右减左每1us有效PWM差的稳态角速度，rad/s |
| speed_time_constant_sec | 2.7 | 前进惯性响应时间常数 |
| yaw_time_constant_sec | 1.1 | 仿真/拟合使用的转向响应时间常数 |
| cruise_speed_m_s | 0.45 | 第二轮将参考巡航速度由0.35提高至0.45m/s |
| maximum_command_percent | 35 | 导航请求上限，之后还经过PWM全局限幅 |

控制使用直线路线前视点→目标偏航角速度→模型前馈+IMU角速度反馈；角差大时降前进速度。
第二轮（162529日志）增加掉头阶段：角差超过0.8rad进入低速对准，前进目标上限0.20m/s；
仅当角差小于0.25rad且角速度小于0.15rad/s时退出。掉头阶段用0.5秒角速度预判提前减小差速；
巡航恢复原有角速度环。日志新增motion_phase、braking_heading_error_rad。
保留20%/秒动力变化率限制，避免对准后突然加速。
左1右2启动值从当前执行器配置核对为1190/1100us（右侧已不是此前1050）。
当前65%全局限幅、等脉宽增量得到满请求增量550us。
因此 `command_span_us=550`。改变执行器启动值、全局限幅或微调后，核对这些映射参数；
预测读取 `/actuators/pwm_state` 的最终脉宽，日志保留请求、限幅后百分比、脉宽和控制板角度。
这些是软件下发值，不是示波器测得的脉宽，也不等同实测推力。

## 日志：用于多轮拟合与人工修正

2026-10-01的S形摆动分析记录：
`/home/hai/Downloads/usv_navigation_logs/20261001_160703_185629/TUNING_REVIEW.md`。
主因是全局IMU符号取反造成正反馈。已修正符号并应用该日志拟合的速度/转向模型，
保持原外环增益；模型对比改善明显，但仍需下一轮实船验证。

可用 `ros2 run usv_goal_navigation navigation_identify <日志目录>` 复现长航次的模型辨识。

每次节点启动自动创建一个会话目录，**手动期间也记录**，避免丢失切换前的历史。

```text
/home/hai/Downloads/usv_navigation_logs/YYYYMMDD_HHMMSS_ffffff/
  parameters.json             本次导航参数
  source_configurations.json  PWM/UDP/IMU/遥控的源配置快照
  telemetry.jsonl             全量数据：每行一个事件
  run_0001.json               第一航次结束摘要（之后编号递增）
```

每次切自动产生run_start、单独run_id；切手动、到达、退出产生run_end。
数据每秒刷新到磁盘，航次结束额外刷新，正常Ctrl+C会关闭日志。会话可包含多个航次。
节点运行时各配置改变的ROS参数事件也记录；断电仍可能丢失末尾缓存/文件系统缓存。

记录内容：

| 事件 | 数据 |
|---|---|
| udp | 接收端单调/Unix纳秒时间、Mac源IP/端口、原始字节hex、包长度、标签→船体转换参数；包括来源匹配但协议损坏/超长的数据 |
| observation | UDP节点解析后的完整场地、球门、船、球、session、seq、frame、Mac时间戳及有效性 |
| imu | ROS时间戳、四元数、三轴角速度、三轴加速度、协方差，逐消息记录 |
| pwm | 请求/实际下发百分比、脉宽、角度、急停、看门狗、来源、dry_run |
| rc / mode / arbitrated_command | 遥控通道、控制权切换和仲裁最终动力 |
| visual | 原始船位、航向、视觉速度、历史预测、位置/航向创新量、修正后的当前状态、是否拒绝 |
| tick | 预测船位/航向/速度、参考点、期望速度/角速度、横向误差、距离、视觉年龄、建议动力、模式和故障 |
| parameter_event | 运行时参数调整事件，例如网页修改限幅 |

日志中的本机monotonic用于对齐不同ROS回调；UDP另外记录socket读取时刻。
这是应用层读取时刻而非网卡硬件接收时间。ROS消息时间戳、Unix和Mac时间原样保留。
UDP完全丢失的包无法记录原文，只能从seq间隔推断缺失；ROS日志传输也可能丢样，不能视为无损黑匣子。

航行结束后可直接提供此目录让我分析，或运行：

```bash
ros2 run usv_goal_navigation navigation_analyze /home/hai/Downloads/usv_navigation_logs/实际会话目录
```

生成 `navigation.csv`、`visual.csv`、`analysis.json`、`candidate_parameters.yaml`；候选参数**不会自动覆盖配置**。
分析将视觉航向增量与IMU积分按0~2秒延迟扫描对齐；
利用实际有效PWM和实测角速度/视觉速度拟合一阶模型的速度/转向增益，并汇总路线误差。
缺少速度变化、左右转向或角速度接近常数时，延迟和增益不可辨识，结果标记不可用。
当前拟合固定时间常数，增益/常数/执行器额外延迟可能耦合，需要多轮不同动作验证。

能估计的是**视觉运动相对本船运动的总滞后**，包含相机缓存、检测和传输；
协议processed_unix_ms不是曝光时间，未同步时钟不能据此单独计算网络时延。
绝对位置偏移也不能仅凭这两类数据唯一确定；标签安装偏移、漂流、航向偏置、延迟可相互混淆。
完整日志为后续联合拟合保留证据，初版不会把创新量均值冒充真实位置偏移。

首轮建议用手动完成短直行、不同油门、左右转弯，再做同一路线两次自动导航。
每轮尽量只调一类参数：先验证航向符号/标签偏置，再速度和转向增益，再惯性与时延。

## 离线模拟与静止参考

无需摄像机、UDP或电机，可运行纯离线模拟：

```bash
ros2 run usv_goal_navigation navigation_simulate --side left
ros2 run usv_goal_navigation navigation_simulate --side right
```

模拟使用假设的一阶船体、500ms视觉滞后、4秒丢包，保存simulation.csv和独立摘要；
它检验算法链路，不证明真实船必然按该路线运动。

IMU节点运行时采集静止参考：

```bash
ros2 run usv_goal_navigation navigation_imu_reference --seconds 30
```

2026-10-01已采集30秒、3001条，ROS接收约100Hz；三轴陀螺读数均为0，
加速度均值约[0.580,-0.132,9.686]m/s²，有小幅噪声。
参考路径：`/home/hai/Downloads/usv_navigation_logs/imu_reference/20261001_145137_094695/`。
这证明该段ROS采样情况，不证明100Hz独立硬件新样本或运动时无偏差；未执行硬件校准，未自动扣除水面漂动。

## 手动示范采集与分析（单向电调）

目的：记录人如何边转向边加速，以及何时减小差速、提前抑制转动惯性，供后续离线分析修改控制器。
**目前是采集和统计分析，不会自动训练、替换控制器或修改任何动力参数。**

### 一键启动

先停止原来的联合导航/追球脚本，避免重复启动串口、PWM和网页节点。
船在安全水域，CH5先保持急停，CH8保持低位手动，油门收零：

```bash
cd /home/hai/usv_project
./start_manual_demonstration.sh left
# 前往右球门则把 left 改为 right
```

此脚本启动真实遥控/PWM、IMU、UDP、网页及采集节点，不启动自动导航控制器。
Mac应正常发送本船位置和场地图；网页仍在 `http://<Jetson-IP>:8080`。
检查状态后解除CH5急停，CH2控制油门、CH4转向，CH8一直保持低位。

在运行脚本的终端按键（无需回车）：

| 按键 | 功能 |
| --- | --- |
| S | 开始新示范；检查手动模式、RC/IMU/PWM新鲜度、有效位置和球门，并读取运行PWM参数 |
| E | 到达球门后标记“人工确认成功”，结束本段并自动分析 |
| X | 结束但标记为非成功，保留数据用于排查 |
| Q / Ctrl+C | 退出采集；一键脚本同时结束自己启动的节点 |

**S/E/X只控制记录，不发动力、不自动停车。E之后仍是手动控制，请收油门或使用CH5急停。**
可以多次S→E采集多条轨迹。切换自动、RC/IMU/PWM/模式数据超时、急停或动力参数变更，
会结束并标记当前示范为中断/无效，避免混入不同控制条件。视觉中途丢帧不会阻断手动控制，
原始记录保留缺测情况；离线统计不插补超过0.5秒的视觉空档。

如果联合控制已经运行，不要再次运行一键脚本，只在另一个交互终端启动记录器：

```bash
cd /home/hai/usv_project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run usv_goal_navigation manual_demonstration --ros-args \
  --params-file /home/hai/usv_project/ros2_ws/src/usv_goal_navigation/config/manual_demonstration.yaml \
  -p goal_side:=left
```

SSH需要交互终端。该记录器不发布任何动力指令；单独退出它不停止其他终端运行的遥控系统。

### 左右独立标定与保存位置

每次S读取 `/pwm_actuator/get_parameters`，保存左右启动阈值、限幅、微调、等增量模式等运行值。
例如左1190us、右1100us，实际下发1290/1200us，表示各自启动阈值以上都增加100us，
而不是因为原始PWM相差90us就判断为转向。阈值随你设置，以每段实时快照为准。
同时保存请求百分比、执行器处理后的百分比、实际下发PWM、控制板角度，并校验软件映射一致性。
“下发PWM”不是示波器测量值；相同增量也**不证明真实推力相同**，必须结合运动响应分析。

每次采集保存到 `~/Downloads/usv_manual_demonstrations/年月日_时分秒_微秒/`：

- `telemetry.jsonl`：带本地时间的完整RC、IMU、动力指令、PWM状态、模式、UDP原始诊断和视觉观测。
- `demonstration.json`：起终点、球门地图、成功标签、结束原因、运行PWM参数和时间对齐假设。
- `parameters.json`、`source_configurations.json`：采集配置和相关源码YAML快照；运行值以demonstration.json为准。
- `demonstration.csv`：按PWM时刻对齐的杆量、各侧PWM增量、角速度、加速度、位置、速度和球门角度误差。
- `analysis.json`：有效样本、缺测比例、映射异常、加油时的角度误差、速度分布、差速与角速度统计。

视觉按配置 `measurement_delay_sec` 向前对齐，保留UDP接收时间、视觉原始时间戳和ROS头，
后续可重新估计延迟；当前不宣称已测出跨设备绝对延迟。差速分组是描述性统计，不是已辨识的转向增益。
E只是人工成功标签，并非程序确认进门；质量标记仅供筛选，不能直接证明示范适合学习。

手动重新分析：

```bash
ros2 run usv_goal_navigation manual_demo_analyze /home/hai/Downloads/usv_manual_demonstrations/具体时间目录
```

修改 `config/manual_demonstration.yaml` 后重启记录器即可；更换电调阈值应先按原来的PWM节点参数流程生效，
再开启新示范。后续把采集目录交给我，可对照运动响应和加速时机调整导航，不会凭一条轨迹直接认定最优参数。

### 本次新增代码首次构建

```bash
cd /home/hai/usv_project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select usv_goal_navigation --symlink-install
source install/setup.bash
```

## 实现文件

`navigation_node.py`负责ROS、状态机和记录；`estimation.py`负责地图缓存/延迟重放；
`control.py`负责路线跟踪；`analysis.py`负责离线拟合；`simulation.py`负责模拟。
原版遇失效观测立即清空状态的node.py已移除，执行入口已切到新实现。
