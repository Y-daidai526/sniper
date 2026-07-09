# Sniper — 低带宽图传适配 RoboMaster 裁判系统 0x0310

将 H264 低带宽视频压缩流适配到 RoboMaster 裁判系统图传链路，替换原有的 ROS2 Topic 通信。

![](https://img.shields.io/badge/ROS2-Jazzy-blue) ![](https://img.shields.io/badge/Python-3.12+-green) ![](https://img.shields.io/badge/C++-17-orange)

---

## 1. 环境准备

### 1.1 系统依赖

```bash
# GStreamer 1.0
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools

# OpenCV
sudo apt install -y libopencv-dev

# mosquitto (本地调试用)
sudo apt install -y mosquitto mosquitto-clients
```

### 1.2 MVS SDK（海康相机）

```bash
# 下载并安装 MVS SDK：https://www.hikrobotics.com/machinevision
# 默认安装路径 /opt/MVS/
ls /opt/MVS/include/MvCameraControl.h  # 确认 SDK 已安装
```

### 1.3 Python 依赖

```bash
# 发送端 debug 模块 + 接收端
pip install --break-system-packages paho-mqtt protobuf av opencv-python
```

---

## 2. 编译

```bash
cd /home/y/sniper
# 首次需先编译消息定义
colcon build --packages-select doorlock_sniper
# 编译发送端和接收端
colcon build --packages-select sender receiver
source install/setup.zsh
```

---

## 3. 运行

> **所有节点均通过 `ros2 launch` 启动**，不要使用 `ros2 run`。
> 发送端的相机和编码器需要 ComposableNodeContainer 实现零拷贝，`ros2 run` 无法正确加载。

### 3.1 仅发送端 + 原解码器（本地 ROS2 调试验证管线）

不需要串口、不需要 MQTT，直接通过 ROS2 Topic 验证编码管线：

```bash
# 终端1: 启动发送端（相机 + 编码器，零拷贝）
ros2 launch sender sniper.launch.py

# 终端2: 启动原解码器（来自 Pacific_doorlock_sniper）
source /home/y/sniper/Pacific_doorlock_sniper/install/setup.zsh
ros2 run doorlock_decoder decoder_node --ros-args -p width:=300 -p height:=300
```

### 3.2 发送端 + MQTT 接收端（本地端到端）

```bash
# 终端1: 启动本地 mosquitto
mosquitto -p 3333

# 终端2: 启动发送端
ros2 launch sender sniper.launch.py

# 终端3: 启动接收端
ros2 launch receiver receiver.launch.py
```

发送端输出三路：
- ROS2 `/video_stream` (150B VideoPacket)
- USB CDC 串口（309B 帧，热插拔）
- debug bridge → MQTT

### 3.3 真实部署（通过裁判系统）

```bash
# 机器人端
ros2 launch sender sniper.launch.py

# 操作手PC（修改 receiver_params.yaml 中的 broker_host 为裁判系统 IP）
ros2 launch receiver receiver.launch.py
```

---

## 4. 调试技巧

### 查看统计信息

发送端每秒输出 TX stats（ros2/serial 两个 buffer 的窗口字节数、backlog 大小）：

```
[video_encoder] TX: ros2=10.95kB serial=11.10kB (limit=14.00kB) backlog ros2=134B serial=70B
```

- `ros2=10.95kB` / `serial=11.10kB` — 过去 1 秒窗口内 ROS2/串口路径的发送量
- `limit=14.00kB` — 窗口上限 (bandwidth_limit_kbytes × bandwidth_window_s)
- `backlog` — 各路径缓冲区中尚未发出的 H264 字节

接收端每秒输出 stats（包速率、数据速率、丢包率）：

```
[stats] packets=1500 rate=300pkt/s 87.6kB/s gaps=3 dups=0 loss=0.2%
```

### 验证 CRC

```bash
cd sender/debug
python3 -c "
from serial_parser import crc8, crc16
header = bytes([0xA5, 0x2C, 0x01, 0x00])
print(f'CRC8: 0x{crc8(header):02X}')
"
```

### 调试图片

在 `sender/config/sender_params.yaml` 中设置 `debug_dump_enable: true`，每 N 帧保存编码器 4 个窗口的 PNG 到 `sniper_debug_imgs/encoder/`。

### 串口抓包

```bash
ls /dev/ttyACM* /dev/ttyUSB*
cat /dev/ttyACM0 | xxd -c 309
```

---

## 5. 参数调优

所有参数在 YAML 配置文件中修改，无需改代码。修改后重新 launch 即可生效。

### 发送端（`sender/config/sender_params.yaml`）

**编码参数**

| 参数 | 默认值 | 效果 |
|------|--------|------|
| `target_bitrate` | 80 kbps | 增大→画质提升、带宽增加。低码率模式(≤80)自动优化 |
| `output_size` | 300 px | 缩小→编码像素少、带宽降低 |
| `output_fps` | 60 | ≤60，降低→减少编码帧数 |
| `x264_preset` | veryslow | ultrafast→延迟低压缩率差；veryslow→压缩率高 CPU 占用大 |
| `bandwidth_limit_kbytes` | 14.0 kB/s | 硬限速上限，同时作用于 150B 和 300B 路径 |
| `bandwidth_window_s` | 1.0 s | 限速滑动窗口长度 |

**图像预处理**

| 参数 | 默认值 | 效果 |
|------|--------|------|
| `crop_size` | 800 px | 从 1624×1240 中心裁剪的尺寸 |
| `static_simplify` | true | 开启后静态背景高斯模糊，大幅度降低码率 |
| `bg_blur_sigma` | 1.8 | 越大→静态区越模糊→码率越低 |
| `motion_threshold` | 14 | 越小→更多像素判为运动→码率越高 |
| `motion_trail_frames` | 90 (clamped to 15) | 运动拖影帧数，0=关闭 |
| `center_clear_size` | 150 px | 中心保护区不模糊，保证瞄准精度 |

**相机参数**

| 参数 | 默认值 | 效果 |
|------|--------|------|
| `exposure_time` | 12000 us | 曝光时间，越大画面越亮 |
| `gain` | 10.0 | 增益 |

### 接收端（`receiver/config/receiver_params.yaml`）

| 参数 | 默认值 | 效果 |
|------|--------|------|
| `broker_host` | 127.0.0.1 | MQTT broker IP |
| `broker_port` | 3333 | MQTT broker 端口 |
| `topic` | CustomByteBlock | MQTT 订阅 topic |
| `width` | 300 | 解码期望宽度 |
| `height` | 300 | 解码期望高度 |
| `display_scale` | 2 | 显示缩放 (300→600) |

### 串口参数（固件约定，不可调）

| 参数 | 值 |
|------|-----|
| 波特率 | 921600 bps |
| 帧大小 | 309 B |
| 有效负载 | 299 B H264/帧 |

---

## 6. 目录结构

```
sender/          C++ ROS2 发送端（机器人端）
  camera/        海康 MVS SDK 相机采集 (ComposableNode)
  encoder/       GStreamer H264 编码 + 双路分包 (ComposableNode)
  protocol/      CRC8/CRC16 + 309B 串口帧构造
  serial/        USB CDC 热插拔监控 + termios
  debug/         C++ bridge + Python MQTT 调试脚本
  config/        发送端参数 YAML
  launch/        发送端启动文件
  main.cpp       独立运行入口（encoder + serial + debug）

receiver/        Python ROS2 接收端（操作手PC）
  receiver/
    mqtt_client.py          MQTT 订阅 + Protobuf 反序列化
    video_decoder_node.py   PyAV H264 解码 + OpenCV 显示
    stats_tracker.py        包速率/数据速率/丢包率
    proto/                  CustomByteBlock protobuf
  launch/         接收端启动文件
  config/         接收端参数 YAML
  main.py         入口
```

---

## 7. 架构说明

详见 [AGENTS.md](AGENTS.md) — 需求架构文档与裁判系统通信协议细节。
