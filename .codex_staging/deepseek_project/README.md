# Jetson DeepSeek 对话程序

## 安装

```bash
cd /home/hai/Deepseek_Project
./install.sh
```

## 安全设置 API Key

不要把 API Key 写入代码或提交到 Git：

```bash
read -s -p "DeepSeek API Key: " DEEPSEEK_API_KEY
echo
export DEEPSEEK_API_KEY
```

## 启动

```bash
cd /home/hai/Deepseek_Project
./start_chat.sh
```

输入 `/clear` 清空对话上下文，输入 `/exit` 退出。

可选环境变量：

- `DEEPSEEK_MODEL`：模型名称，默认 `deepseek-flash`
- `DEEPSEEK_BASE_URL`：接口地址，默认 `https://api.deepseek.com`

下一阶段会将客户端封装为 ROS 2 服务。任何影响推进器或系统的模型输出，
都必须经过固定指令集、类型检查、范围限制、控制模式仲裁和超时保护，不能直接执行。
