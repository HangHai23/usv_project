# 独立硬件验证

这里的程序用于在接入 ROS 2 前独立验证硬件，不属于正式运行系统：

- `imu/read_ybimu_serial.py`：直接读取 Yahboom IMU。
- `crsf/crsf_receiver_monitor.py`：实时解析 CRSF 通道和链路状态。
- `vision/yolo_usb_camera_capture.py`：在局域网预览USB摄像头并采集YOLO原始图像。

串口验证程序和对应 ROS 2 节点不能同时打开同一个串口。
