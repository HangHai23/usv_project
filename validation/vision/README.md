# USB摄像头YOLO图像采集

默认保存到 `~/Downloads/usv_yolo_dataset/images/raw/<启动时间>/`。

```bash
python3 ~/usv_project/validation/vision/yolo_usb_camera_capture.py
```

同一局域网设备打开 `http://<Jetson-IP>:8090`，IP可用 `hostname -I` 查看。

- `S`：开始/暂停连续采集
- `C`：保存单帧
- `Q`：退出（仅Jetson终端）

`S`、`C` 同时支持Jetson终端和Windows网页键盘，也可点击网页按钮。默认连续
采集5张/秒，以减少大量相似帧。

```bash
python3 ~/usv_project/validation/vision/yolo_usb_camera_capture.py \
  --camera auto --width 1280 --height 720 \
  --camera-fps 30 --capture-fps 5 --port 8090
```

程序会尝试所有 `/dev/video*` 并选择能实际输出图像的节点，摄像头掉线后自动
重新扫描。需要固定设备时可用 `--camera /dev/video0`。
