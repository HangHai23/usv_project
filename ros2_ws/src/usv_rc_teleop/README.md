# usv_rc_teleop

当前遥控差速控制映射：

```text
CH2 -> 前后油门
CH4 -> 左右转向
CH5 -> 底层急停（>1500 us 急停，<1500 us 解除）
CH8 -> 控制权（低位遥控，高位自动导航）
PWM通道1 -> 左电机
PWM通道2 -> 右电机
```

混控层只读取 CH2 和 CH4；PWM执行层另行直接检查 CH5。CH1、CH3、
CH9～CH16及其他未使用通道不会影响
动力输出。前进时两侧输出均不低于1500 us，后退时均不高于1500 us；转向
通过降低内侧动力实现，最大转向可将内侧降至中位，但不会反转。油门进入中位
死区时，转向改为一侧前进、一侧后退的原地旋转。

输入方向、电机方向、中位死区和通道编号集中在
`config/rc_differential.yaml`。默认均不反向：

```yaml
throttle_reversed: false
steering_reversed: false
left_motor_reversed: false
right_motor_reversed: false
```

接收机失联或消息过期时两侧输出0%，PWM节点另有独立命令超时回中保护。
PWM节点启动时默认保持急停；只有收到有效的 CH5 低电平才放行动力。

CH8低位时，仲裁节点只把 `/propulsion/manual_command` 转发到最终的
`/propulsion/command`；CH8高位时只接受自动导航节点发布的
`/propulsion/automatic_command`。自动控制尚未发布、任一命令源超时或
遥控器断联时，最终输出均为两路0%。当前模式发布在
`/control/automatic_mode`。

首先使用 dry-run：

```bash
ros2 launch usv_rc_teleop channel3_esc1_test.launch.py dry_run:=true
```

真实示波器测试（默认最终限幅100%）：

```bash
ros2 launch usv_rc_teleop channel3_esc1_test.launch.py
```

必须保证螺旋桨区域安全，并在启动时将 CH2 油门置于中位。启动文件名暂时保留
`channel3_esc1_test.launch.py` 以兼容已有运行命令，但内部已使用双电机差速节点。
