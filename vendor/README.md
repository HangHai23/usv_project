# 第三方源码

此目录保存厂家或上游原始代码，便于追溯和重新安装。正式功能修改应在
`ros2_ws/src` 完成，不应直接把 `vendor` 加入 colcon 工作空间。

- `actuator/16CServo-uart-original.py`：PWM 控制板厂家/原始验证程序。
