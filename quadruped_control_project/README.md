# 四足机器人遥控项目

复用原工作区 CRSF 接收驱动、CH2 油门映射及 I²C PWM 板写入协议。
四条腿各一个舵机：左前1、左后2、右前3、右后4（驱动板1起始编号）。
新增 quadruped_rc_control，直接订阅 /rc/channels，输出四路舵机角度。

## 构建和运行

```bash
cd /home/hai/usv_project/quadruped_control_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch quadruped_rc_control teleop.launch.py
```

默认 dry_run=true，只发布 /quadruped/servo_angles，不写驱动板；仍需连接接收机串口。
真实输出：

```bash
ros2 launch quadruped_rc_control teleop.launch.py dry_run:=false
```

依赖 ROS 2 Humble、colcon、python3-serial、python3-smbus。
接收机默认 /dev/ttyTHS1、420000 baud，驱动板 /dev/i2c-7、地址0x2D。
同一串口和驱动板应只有本项目控制进程，运行前停止原无人艇控制节点。

## 控制

CH2：1000～2000 us 映射为反向～正向，中位1500±20 us 停止并回中。
油门大小控制步态频率，满油门默认1 Hz；负油门反向遍历整个周期。
CH5 >1500 us 急停，<1500 us 解除，等于阈值保持状态。
启动默认急停，失联、消息过期或超过0.2秒无消息时四腿回中。
正常退出回中；进程被强制杀死、主控断电时软件无法回中。

## 步态及校准

原工作区没有四足步态，新实现对角腿配对周期：左前/右后同相，左后/右前相差半周期。
支撑行程占70%，回摆占30%；反向运行交换行程顺序。
单自由度机构不能主动独立抬脚，实际行走方向和效果依赖腿形及足端摩擦，需实机验证。
停止及启动会直接切换至中位或步态角度，目前没有平滑过渡。

配置源文件 ros2_ws/src/quadruped_rc_control/config/quadruped.yaml：
servo_channels、centers_deg、amplitudes_deg、directions、phase_offsets 均按左前、左后、右前、右后排列。
默认中位87°、摆幅±20°、右侧反向为初始假设，需按装配校准。
修改配置后重新构建，或通过 config:=绝对路径直接指定配置文件。
config/leg_channels.yaml 仅记录通道分配，实际运行使用 quadruped.yaml。

查看输出：`ros2 topic echo /quadruped/servo_angles`。
