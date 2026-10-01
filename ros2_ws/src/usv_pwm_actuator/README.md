# usv_pwm_actuator

将最终仲裁后的两路有符号动力百分比转换为 16 路 PWM 控制板的角度命令。
默认使用 Jetson 40 针接口的 I²C 总线 `/dev/i2c-7`，控制板地址为 `0x2D`；
同时保留独立 UART 后端。

## 转换

| 动力 | PWM | 控制器角度 |
|---:|---:|---:|
| `-100%` | 1000 us | 44° |
| `-50%` | 1250 us | 约 66° |
| `0%` | 1500 us | 87° |
| `+100%` | 2000 us | 130° |

输入话题：`/propulsion/command` (`usv_interfaces/msg/PropulsionCommand`)。
状态话题：`/actuators/pwm_state` (`usv_interfaces/msg/PwmOutputState`)。

节点启动时回中；超过 `command_timeout` 未收到命令时回中；退出时再次回中。
`/dev/ttyTHS1` 已用于 420000 baud 的 CRSF，不能再作为 PWM 控制板的 9600-baud
UART。当前正式配置使用 I²C，避免两者争用。

## 底层急停

PWM执行节点直接订阅 `/rc/channels` 并检查 CH5，不依赖上层遥控混控或导航
命令。CH5 `>1500 us` 时立即将两路强制写为1500 us（约87°）；CH5
`<1500 us` 时解除。恰好等于阈值时保持原状态。节点启动、接收机失联或
RC通道数据超时均保持急停状态。

## 动力限幅和左右微调

在 `config/pwm_actuator.yaml` 中调整：

```yaml
output_limit_percent: 60.0
channel_1_trim_percent: 0.0
channel_2_trim_percent: -5.0
```

`output_limit_percent` 是 PWM执行层的最终全局限制，应用于遥控、自动导航及
未来所有上层命令。联合启动文件不会覆盖该值。比如设置为50%时，两路最终
脉宽都只能处于1250～1750 us之间（急停/超时时为1500 us）。

运行时可直接修改，无需重新构建或重启：

```bash
ros2 param set /pwm_actuator output_limit_percent 50.0
ros2 param set /pwm_actuator channel_1_trim_percent 0.0
ros2 param set /pwm_actuator channel_2_trim_percent -5.0
```

动态修改只对当前进程有效；需要重启后仍保留时，再同步修改 YAML。

## 电调启动死区补偿

左右电调可分别配置正反向稳定启动脉宽：

```yaml
channel_1_forward_start_us: 1560  # 左电机正向
channel_2_forward_start_us: 1520  # 右电机正向
channel_1_reverse_start_us: 1440  # 左电机反向，需实测
channel_2_reverse_start_us: 1480  # 右电机反向，需实测
```

保持1500表示关闭该方向补偿。零动力始终输出1500 us；非零动力从对应启动
脉宽向全局限幅允许的端点线性映射。全局限幅始终拥有最高优先级，例如50%
限幅下输出仍严格限制在1250～1750 us。如果限幅端点尚未达到某侧实测启动
脉宽，该侧不会被补偿到限幅以外，因此可能无法启动，这是预期的安全行为。

处理顺序：输入 `-100..100%` 先线性映射到总限幅范围，再乘以通道微调增益，
最后再次限制在总限幅以内。上例中输入 `50%` 的基础输出为 `30%`，通道2
微调后为 `28.5%`。比例微调不会改变 `0%` 中位状态。

干运行：

```bash
ros2 launch usv_pwm_actuator pwm_actuator.launch.py dry_run:=true
```

真实硬件：

```bash
ros2 launch usv_pwm_actuator pwm_actuator.launch.py
```

默认参数等价于：

```bash
ros2 launch usv_pwm_actuator pwm_actuator.launch.py \
  backend:=i2c i2c_bus:=7 i2c_address:=45
```

如以后连接了真正独立的 UART：

```bash
ros2 launch usv_pwm_actuator pwm_actuator.launch.py \
  backend:=uart port:=/dev/ttyUSB0
```
