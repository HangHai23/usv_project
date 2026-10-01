# YOLO11红球模型

- `best.onnx`：YOLO11n，输入 `1x3x640x640`，类别 `0: redball`。
- `best.engine`：在当前Jetson Orin Nano Super上使用TensorRT 10.3生成的FP16、batch=1 Engine。
- `validation_results/`：四张验证图片的检测框可视化结果。

该Engine与当前JetPack/TensorRT/GPU环境绑定，不应复制到不同平台使用。升级JetPack或TensorRT后，应从ONNX重新构建。

2026-08-22离线验证：四张图片均检出一个红球，置信度分别为0.9299、0.9271、0.9219和0.9239；稳定TensorRT推理约13 ms/帧。

ROS 2配置位于：

```text
ros2_ws/src/usv_yolo_detector/config/yolo_detector.yaml
```
