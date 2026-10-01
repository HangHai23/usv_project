# Mac视觉UDP接收（air2s.vision v1）

依据项目根目录 jetson_udp_handoff.zip（2026-09-28）适配。旧版5005端口的x/y/stamp协议已替换。

## 配置与启动

编辑 config/udp_position.yaml：
- allowed_sender_ip：必须填写Mac的IPv4地址，空字符串拒绝所有包。
- boat_id：本船AprilTag ID，默认0需要核实。
- tag_to_body_heading_deg：标签0/1边方向到船头的逆时针偏置角。
- tag_to_body_xy_m：标签中心到定位参考点的向量，船头X、左侧Y，米。
- 与IMU位置预测融合时，定位参考点设为IMU位置。

```bash
cd /home/hai/usv_project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch usv_udp_position udp_position.launch.py
```

Mac向Jetson的UDP5700发送完整UTF-8 JSON快照；源端口不固定。
每包最大1400字节，协议必须为air2s.vision，version为整数1。

## 发布接口

- /vision/global_observation：std_msgs/String，JSON完整有效场地快照及own_boat修正位置、方向和拿球状态。
- /usv/global_position_valid：std_msgs/Bool，本船位置方向在有效场地快照中可用；不表示时钟和安装已标定。
- /usv/global_position：原有GlobalPosition接口；仅publish_position=true时发布，每个视觉frame最多一次。

```bash
ros2 topic echo /vision/global_observation
ros2 topic echo /usv/global_position_valid
```

快照包括球、船、球门、场地尺寸；缺项不沿用旧目标，held只是视觉拿球状态，不能当机械捕获确认。
场地X向右、Y向上、单位米，不是GPS/ENU。保留coord身份，不自动当作已标定的导航map。

## 时间与位置预测对接

processed_unix_ms是检测结束时刻，不是曝光时刻。age_ms从Mac解码完成计时，
不包含此前图传延迟。默认publish_position=false，仅查看观测。
实测验证两机时钟同步后设置clock_synchronized=true，并测量
processed_to_measurement_ms（处理结束到真实测量时刻的延迟），然后开启publish_position。
0补偿只是处理时刻近似，不能解释为已准确恢复采样时刻。
同步模式会拒绝超过maximum_network_delay_ms的网络迟到包和明显超前包。

接收后按min(500ms, ttl_ms-age_ms)设置本地单调时钟有效期。
未同步模式无法发现包到达之前的排队延迟。未知版本、非法字段、非有限数、重复/旧seq拒绝。
相同frame不延长原有效截止时间。定时器独立检查断流，失效包、本船缺失立即发布false。

发送进程session或标定coord改变时，接收器锁定reset_required=true并清空观测。
需确认新坐标系，重启UDP接收器和位置预测节点，清空旧轨迹/路径后再使用。
目前位置预测节点未订阅此valid话题，仍按其position_timeout判断旧位置新鲜度；
本次改动不接入自动动力，不可依赖位置预测valid实现即时视觉急停。

## 验证

```bash
PYTHONPATH=src/usv_udp_position python3 -m pytest src/usv_udp_position/test -q
```

仍需现场验证Mac来源IP、标签偏置、坐标对齐、时间补偿及无线延迟。
