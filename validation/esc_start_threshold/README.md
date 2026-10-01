# 单向电调启动阈值测试

本项目只测试一个电机在指定PWM脉宽下是否能够启动。不执行直行、左右转向、
增益拟合或自动参数修改。另一侧电机始终为1000us停机。

正式执行器仍采用单向模式：1000us停机，2000us为100%前进，负动力截为零。

## 启动节点

不得同时运行正式追踪、正常遥控launch或其他PWM节点。
每个终端先执行：

```bash
cd /home/hai/usv_project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

终端1：

```bash
ros2 launch usv_imu_driver imu.launch.py
```

终端2：

```bash
ros2 launch usv_crsf_receiver crsf_receiver.launch.py
```

终端3启动真实PWM测试执行器：

```bash
ros2 launch /home/hai/usv_project/validation/esc_start_threshold/actuator.launch.py dry_run:=false
```

遥控器CH5高位为急停，低位允许测试。CH2、CH4、CH8不参与本测试。

## 单次测试

CH5先置低位，然后另开终端。例如测试左电机1050us：

```bash
python3 /home/hai/usv_project/validation/esc_start_threshold/start_threshold_test.py \
  --channel 1 --pulse-us 1050 --duration 3
```

右电机：

```bash
python3 /home/hai/usv_project/validation/esc_start_threshold/start_threshold_test.py \
  --channel 2 --pulse-us 1050 --duration 3
```

程序只保持指定输出3秒，然后恢复两侧1000us。随后在终端输入：

- `1`：稳定启动。
- `2`：仅抖动、偶尔启动或不稳定。
- `3`：完全未启动。

每次确认电机停稳后，再提高脉宽进行下一次测试。出现异常立即拨高CH5。

每次测试结果保存为独立YAML：

```text
/home/hai/Downloads/usv_start_threshold/
```

把该目录路径告诉助手即可读取结果。启动成功必须由现场观察确认，IMU数据只作为
辅助记录，不用于自动判断电机是否启动。

若只想检查流程、不输出真实PWM：

```bash
ros2 launch /home/hai/usv_project/validation/esc_start_threshold/actuator.launch.py dry_run:=true
```
