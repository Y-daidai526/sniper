# RoboMaster2026 自定义客户端图传

本仓库包含机器人端 `sender/` 和操作手端 `receiver/` 两个 ROS 2 包。

## 1. 安装依赖

### 海康 MVS SDK

安装海康 MVS SDK，并确认 `/opt/MVS/include` 和 `/opt/MVS/lib/64` 存在。

### apt

```bash
sudo apt install -y \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  gstreamer1.0-tools libopencv-dev \
  python3-pip python3-av python3-opencv
```

### 安装 ROS 相关依赖

```bash
rosdepc install --from-paths sender receiver --ignore-src -r -y
```

### pip

```bash
pip install paho-mqtt==2.1.0 amqtt
```

Python 3.12 若因 PEP 668 禁止直接使用 `pip` 安装系统包，请使用：

```bash
pip install --break-system-packages paho-mqtt==2.1.0 amqtt
```

## 2. 运行 sender

配置文件：`sender/config/sender_params.yaml`。

编译并运行：

```bash
./start_sender.sh
```

需要在具备图形环境的终端显示 sender 四宫格预览时：

```bash
./start_sender.sh show:=true
```

程序流程：

- 从海康相机获取图像
- 裁剪、缩放、静态背景简化、运动模糊
- 编码为 H264 码流
- 每 299B 分片，头部添加1字节内部序号
- 封装为 RoboMaster 0x0310 帧（309B）
- 通过串口发送，控制发送速率
- 同时发送到本机 MQTT 用于调试

可用于调试的话题：

```text
/sender/image_raw       sensor_msgs/Image           检查相机采集的原始 BGR8 图像和帧率
/sender/camera_info     sensor_msgs/CameraInfo      检查相机标定信息
/sender/video_preview   sensor_msgs/Image           检查 sender 四宫格处理预览
/sender/encoded_stream  std_msgs/UInt8MultiArray    检查编码器输出的 H264 Annex-B 码流
/sender/rm_frame        std_msgs/UInt8MultiArray    检查封装后的 309B RoboMaster 0x0310 帧
```

## 3. 运行 receiver

配置文件：`receiver/config/receiver_params.yaml`。

编译并运行：

```bash
./start_receiver.sh client_id:=1
```

未传入 `client_id` 时会在启动阶段提示输入。

程序流程：

1. 订阅 MQTT CustomByteBlock话题（QoS=1）
2. 校验内部序号，去除重包后发布 299B H264 码流分片（统计重复包和丢包）
3. 持续解码为原始视频流
4. 缩放并绘制准星

可用于调试的话题：

```text
/receiver/encoded_stream    std_msgs/UInt8MultiArray    检查 MQTT 接收后还原的 299B H264 码流分片
/receiver/video_stream/raw  sensor_msgs/Image           检查未经缩放和准星处理的原始解码图像
/receiver/video_stream      sensor_msgs/Image           检查缩放并绘制准星后的最终图像
```

## 4. 本地 MQTT 联调

### 原因

无需连接机器人和裁判系统，即可在同一台电脑上检查 sender 到 receiver 的完整图传链路。

### 原理

```text
sender RM 帧
  -> 提取 300B data
  -> 封装为 protobuf CustomByteBlock
  -> 发布到 127.0.0.1:3333
  -> sender 内置 amqtt broker
  -> receiver 订阅 CustomByteBlock
  -> 去除 1B 内部序号
  -> 还原 299B H264 码流并解码
```

本地 MQTT 只替代外部 broker 和网络传输，编码、组帧、序号检查及解码流程保持不变。

### 使用方法

确认 `sender/config/sender_params.yaml` 中内置 broker 和 MQTT sender 已启用，地址为 `127.0.0.1`，端口为 `3333`。

将 `receiver/config/receiver_params.yaml` 中的 broker 地址改为：

```yaml
broker_host: "127.0.0.1"
broker_port: 3333
topic: "CustomByteBlock"
```

分别在两个终端启动：

```bash
./start_sender.sh
```

```bash
./start_receiver.sh client_id:=1
```

观察 receiver 窗口或 `/receiver/video_stream`，即可确认整条链路是否正常。
