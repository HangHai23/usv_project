# ExpressLRS / CRSF 接收机验证

接线：接收机 TX 接 Jetson 40Pin 的 RX（物理 Pin 10），两端必须共地。当前
Jetson Orin Nano 上对应设备为 `/dev/ttyTHS1`。

实时运行：

```bash
python3 crsf_receiver_monitor.py
```

进行固定时长、保留终端输出的测试：

```bash
python3 crsf_receiver_monitor.py --duration 5 --no-clear
```

程序按 CRSF 的 420000 baud、8N1 格式打开 UART，执行 CRC-8/DVB-S2 校验，
显示 16 个遥控通道、RC 帧率以及 0x14 链路统计帧中的 RSSI、LQ 和 SNR。

