# AGENTS.md — 低带宽图传适配 RoboMaster 裁判系统 0x0310

## 1. 项目目标

将 [Pacific_doorlock_sniper](Pacific_doorlock_sniper/) 的低带宽 H264 视频压缩数据流，适配到 **RoboMaster 裁判系统图传链路**（命令码 0x0310），替换原有 ROS2 Topic 通信层。

| | 原有 | 目标 |
|---|------|------|
| 传输层 | ROS2 Topic `/video_stream` | 串口(309B帧) → 裁判系统图传链路 → MQTT |
| 消息封装 | `VideoPacket` (166B) | 串口帧 (309B) → Protobuf `CustomByteBlock` |
| 分包大小 | 150B (VideoPacket.msg 硬约束) | **300B** = 1B seq + 299B H264 切片 |
| 接收端 | ROS2 Subscriber | MQTT Subscriber (topic: `CustomByteBlock`) |
| ROS2 调试 | VideoPacket → 原解码器 | **保留** 150B VideoPacket publish，**兼容原解码器** |

> **关于 150→300**: 原项目 `VideoPacket.msg` 定义 `uint8[150]`，C++ 代码强制 `packet_size=150`（launch 文件设 300 被覆盖无效）。0x0310 的 data 段固定 300B。编码器输出的是连续 Annex-B 字节流，按 299B 从 stream buffer 切块（不够就等），**不做末尾补零**。同时保留一路 150B VideoPacket publish，直接兼容原解码器。

**约束**: 不动 `Pacific_doorlock_sniper/`，在根目录新建 `sender/` 和 `receiver/`。

---

## 2. 架构与数据流

```
┌─ sender/ (机器人端, C++ ROS2) ──────────────────────────────────────────────┐
│                                                                              │
│  hik_camera (MVS SDK) ──/image_raw──▶ video_encoder (GStreamer x264enc)     │
│  1440×1080@250fps                      预处理 → H264 Annex-B                 │
│                                        ↓                                    │
│                          appsink 拉取 H264 字节                              │
│                        ╱                       ╲                             │
│           serial_stream_buffer_         ros2_stream_buffer_                 │
│           按 299B 切片                   按 150B 切片                        │
│           + 1B seq = 300B               → VideoPacket publish               │
│                ↓                        → /video_stream                     │
│           referee_serial                (兼容原 doorlock_decoder)           │
│           构造 309B 串口帧                                                   │
│          ╱              ╲                                                    │
│   serial_writer     debug_bridge                                            │
│   USB CDC ttyACM0   fork+exec → stdin                                      │
│   921600 8N1        pipe (309B帧)                                           │
│   (热插拔)                                                                   │
└──────────┬─────────────────┬────────────────────────────────────────────────┘
           │                 │
  真实部署 ▼                 ▼  本地调试
┌──────────────────┐  ┌──────────────────────────────┐
│ 裁判系统硬件      │  │ sender/debug/main.py          │
│ 图传发送端→接收端 │  │ stdin.read(309) → CRC验证     │
│ 裁判系统服务器    │  │ → Protobuf → MQTT publish     │
│ MQTT Broker      │  │                              │
│ 192.168.12.1:3333│  │ mosquitto broker             │
│ (校验 client ID) │  │ 192.168.12.1:3333            │
└────────┬─────────┘  │ (不校验 client ID)           │
         │             └──────────┬───────────────────┘
         │ MQTT                   │ MQTT
         ▼                        ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│ receiver/ (操作手PC, Python ROS2)                                            │
│                                                                              │
│ main.py: 提示 client_id → MQTT连接 192.168.12.1:3333                        │
│ mqtt_client.py: 订阅 CustomByteBlock → Protobuf反序列化 → 提取300B          │
│ video_decoder_node.py: data[0]=seq → 丢包检测 → data[1..]=H264 → PyAV解码   │
│ stats_tracker.py: 包速率/数据速率/丢包率                                     │
└──────────────────────────────────────────────────────────────────────────────┘
```

**核心思路**: appsink 拉出的 H264 字节**同时喂给两个 buffer**：
- `ros2_stream_buffer_` (150B 切片) → VideoPacket → `/video_stream` — **兼容原解码器**，本地 `ros2 run` 即可验证
- `serial_stream_buffer_` (299B 切片 + 1B seq) → 309B 帧 → 串口 + debug stdin — 经 MQTT 到 receiver

---

## 3. 关键设计决策

| 决策 | 选择 | 原因 |
|------|------|------|
| 发送端框架 | C++ ROS2 | MVS SDK + GStreamer C API + intra-process零拷贝 |
| 接收端框架 | Python ROS2 | PyAV (ffmpeg binding)，复用原decoder结构 |
| 分包大小 | **300B** = 1B seq + 299B H264 | 0x0310 data段固定300B |
| 内部序列号 | **uint8** (1B) 嵌入 data[0] | 0-255，50Hz下约5秒回绕，丢包检测够用 |
| H264 切片 | 299B 从 serial_stream_buffer_ 切 | 连续 Annex-B 流，不够就等，不补零 |
| **ROS2 调试** | **保留 150B VideoPacket publish** | 发送端依赖 `doorlock_sniper` 包的消息定义 |
| | → `/video_stream` | **原解码器可直接订阅**，无需任何修改 |
| | → 两个独立 stream buffer | 150B 路径无限速（本地），300B 路径有限速 |
| 输出路径 | 串口(300B路径) + VideoPacket(150B路径) + debug stdin(300B路径) |  |
| Broker地址 | 始终 `192.168.12.1:3333` | 真实部署和本地调试一致 |
| 调试broker | 本地mosquitto，不校验client_id | 本地调试简化 |
| Client ID | 接收端启动时始终提示输入 | 真实部署校验，本地调试不校验但保留流程 |
| IPC机制 | fork+exec → 子进程stdin pipe | 比Unix socket简单，生命周期绑定 |
| 串口热插拔 | **支持**，持续监控 | 插入时打印日志并自动连接，拔出时提示；无串口不崩溃 |
| 参数管理 | sender/receiver **各自独立** YAML | ROS2 标准方式，方便调试时调整 |
| 公共常量 | 无common/目录 | CRC/常量C++在protocol/，Python在serial_parser.py，各自维护 |

### 能否单独驱动原解码器？

**可以。** 发送端把同一路 H264 码流同时切为 150B VideoPacket publish 到 `/video_stream`。原解码器 `video_decoder_node.py` 无需任何修改，直接订阅即可：

```bash
# 终端1: 启动发送端（带相机或测试图片）
ros2 launch sender sniper.launch.py

# 终端2: 启动原解码器（来自 Pacific_doorlock_sniper）
ros2 run doorlock_decoder video_decoder_node --ros-args -p topic:=/video_stream
```

> 此命令写入 README 的运行与调试章节。

这样就可以在**不经过串口、不经过 MQTT** 的情况下，验证编码管线是否正常。开发时调试图像预处理/GStreamer 参数非常方便。

---

## 4. 目录结构

```
/home/y/sniper/
├── AGENTS.md
├── README.md                              # 最后编写：运行与调试指南（含原解码器独立验证命令）
├── docs/
│   └── RM2026通信协议_V2.0.0.pdf
├── Pacific_doorlock_sniper/               # 开源代码 (不动，仅参考)
│
├── sender/                                # 发送端 (机器人端)
│   ├── camera/                            # MVS SDK相机采集 (C++ ROS2 ComposableNode)
│   ├── encoder/                           # GStreamer H264编码 + 双路分包 (C++ ROS2 Node)
│   │   ├── video_encoder_node.cpp/hpp    #   图像预处理 + GStreamer管线
│   │   └── packetizer.cpp/hpp            #   双buffer: 150B→VideoPacket + 299B→serial
│   ├── protocol/                          # CRC8/16 + 串口帧构造 (纯C++)
│   ├── serial/                            # USB CDC热插拔监控 + termios (纯C++)
│   ├── debug/                             # 调试桥接 (C++ bridge + Python MQTT模拟)
│   │   ├── debug_bridge.cpp/hpp          #   C++: fork/exec/stdin pipe
│   │   ├── serial_parser.py              #   Python: 解析309B帧,验证CRC
│   │   ├── mqtt_server.py                #   Python: mosquitto管理 + MQTT发布
│   │   ├── proto/CustomByteBlock.proto   #   Protobuf定义
│   │   ├── main.py                        #   子进程入口
│   │   └── requirements.txt
│   ├── config/
│   │   └── sender_params.yaml             # 发送端参数
│   ├── main.cpp                           # 入口: ROS2 spin, 每帧三输出
│   ├── sniper.launch.py
│   ├── package.xml                        # 依赖 doorlock_sniper (VideoPacket消息定义)
│   └── CMakeLists.txt
│
└── receiver/                              # 接收端 (操作手PC)
    ├── receiver/                          # ROS2 Python package
    │   ├── video_decoder_node.py         #   MQTT回调 → PyAV H264解码 + OpenCV显示
    │   ├── mqtt_client.py                #   MQTT订阅 + Protobuf反序列化
    │   ├── stats_tracker.py              #   包速率/数据速率/丢包率
    │   └── proto/CustomByteBlock.proto
    ├── launch/receiver.launch.py
    ├── config/
    │   └── receiver_params.yaml           # 接收端参数
    ├── main.py                            # 入口: client_id → MQTT → ROS2 spin
    ├── package.xml
    ├── setup.py
    └── requirements.txt
```

> 发送端不自定义新消息类型，直接依赖 `doorlock_sniper` 包使用其 `VideoPacket.msg`，保证与原解码器类型一致。

---

## 5. 协议细节

### 5.1 串口帧格式 (309B)

```
┌──────┬──────────┬──────┬──────┬────────┬──────────┬───────┐
│ SOF  │data_len  │ seq  │ CRC8 │ cmd_id │   data   │ CRC16 │
│ 1B   │ 2B       │ 1B   │ 1B   │ 2B     │  300B    │ 2B    │
│ 0xA5 │ 0x012C   │ 0-255│      │ 0x0310 │          │       │
└──────┴──────────┴──────┴──────┴────────┴──────────┴───────┘
```

- `data_len` = 300 (0x012C)，小端序
- `CRC8` 覆盖 SOF + data_len + seq (前4B)，多项式 `x^8+x^5+x^4+1`，初始值 0xFF，查表法
- `CRC16` 覆盖 SOF 到 data 末尾 (整包减2B)，多项式 `0x1189`，初始值 0xFFFF，小端序，查表法
- 串口参数: 921600 bps, 8N1, 无流控

### 5.2 0x0310 命令

| 属性 | 值 |
|------|-----|
| 方向 | 己方机器人 → 图传链路 → 自定义客户端 |
| data 长度 | **300 字节** (固定) |
| 频率上限 | 50 Hz |
| 重传 | **无** |

### 5.3 MQTT + Protobuf

| 属性 | 值 |
|------|-----|
| Broker | `192.168.12.1:3333` |
| Topic | `CustomByteBlock` |
| Protobuf | `message CustomByteBlock { optional bytes data = 1; }` |

### 5.4 300B 内部数据布局（0x0310 data 段）

```
┌──────────────────────┬────────────────────────────────────────────┐
│ 内部 seq (1B)         │  H264 Annex-B 字节流 (299B)                │
│  uint8_t, 0-255       │  从 serial_stream_buffer_ 按 299B 切块     │
│                        │  不够就等，不补零                          │
└──────────────────────┴────────────────────────────────────────────┘
```

接收端用法：
- `data[0]` = 内部 seq，不连续 → 丢包计数 + 解码器重置
- `data[1..299]` = H264 Annex-B 字节，直接喂 PyAV
- seq 重复 → 丢弃重包；不处理乱序，不缓存重排
- H264 自带 `00 00 00 01` 起始码分隔 NAL 单元，PyAV 自动处理跨包边界

### 5.5 双路分包机制

appsink 拉出的 H264 字节**同时追加到两个独立 buffer**：

| | ros2_stream_buffer_ | serial_stream_buffer_ |
|---|----|----|
| 切片大小 | **150B** | **299B** |
| 封装 | `VideoPacket` (seq_id + ts + data[150]) | 1B seq + 299B H264 → 300B |
| 输出 | ROS2 topic `/video_stream` | 309B 串口帧 → serial_writer + debug_bridge |
| 限速 | 无（本地 ROS2，带宽足够） | 滑动窗口 ~7 kB/s |
| 消费者 | 原 doorlock_decoder (不变) | 裁判系统 / debug MQTT → receiver |

两个 buffer 独立消费，互不干扰。150B 路径走原项目的 `pull_stream_and_packetize()` 逻辑（复制过来稍作调整），300B 路径新增。

---

## 6. 需求要点

1. **串口帧完整构造**: 发送端构造完整 309B 帧（SOF + 帧头CRC8 + cmd_id + data + CRC16），不只是 300B 数据
2. **内部序列号统计**: 1B uint8 seq 嵌入 `data[0]`。接收端基于此统计包速率、数据速率(kB/s)、丢包率(%)、重包检测
3. **发送端保留相机+ROS2 + VideoPacket**: 完整移植 `hik_camera_node` + `video_encoder_node`，保留 ROS2 ComposableNode。双路分包：150B VideoPacket → `/video_stream`（兼容原解码器）+ 300B 路径 → 串口 + debug stdin
4. **USB CDC 热插拔**: 持续监控 `/dev/ttyACM*` 和 `/dev/ttyUSB*`，设备插入时打印日志并自动连接，拔出时提示。无串口不崩溃，编码器继续运行
5. **本地调试程序**: `sender/debug/` — 发送端启动时自动 fork+exec 启动 `main.py`，通过 stdin pipe 传递 309B 帧。Python 端解析 CRC → 提取 data → Protobuf 封装 → MQTT 发布。自动管理 mosquitto broker
6. **子进程IPC**: `fork()` + `exec()` 启动 `python3 sender/debug/main.py`，父进程写 stdin pipe，子进程 `sys.stdin.buffer.read(309)` 阻塞读取。父进程退出时关闭 pipe，子进程 EOF 退出
7. **接收端 Client ID + ROS2**: 唯一版本，Python ROS2 Node。启动时始终提示输入 client_id，始终连接 `192.168.12.1:3333`。MQTT 回调替代 ROS2 Subscription 驱动解码+显示
8. **参数文件分离**: sender 参数在 `sender/config/sender_params.yaml`，receiver 参数在 `receiver/config/receiver_params.yaml`

---

## 7. 关键参数

### 编码参数 (`sender/config/sender_params.yaml`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| crop_size | 800 | 中心裁剪 (px) |
| output_size | 300 | 编码分辨率 (px²) |
| output_fps | 60 | 编码帧率 (≤60) |
| target_bitrate | 40 | x264 目标码率 (kbps) |
| bandwidth_limit_kbytes | 7.0 | 发送带宽上限 (kB/s) — 仅限 serial 路径 |
| bandwidth_window_s | 2.0 | 限速滑动窗口 (s) |
| max_tx_delay_s | 1.0 | 最大发送队列时延 (s) |
| motion_trail_frames | 90 | 拖影历史帧数 |
| bg_blur_sigma | 1.8 | 静态区高斯模糊 σ |
| center_clear_size | 150 | 中心保护区 (px) |

### 协议参数

| 参数 | 值 |
|------|-----|
| 串口波特率 | 921600 bps, 8N1 |
| cmd_id | 0x0310 |
| data_length | 300 B |
| 频率上限 | 50 Hz |
| 串口帧总长 | 309 B |
| MQTT Broker | 192.168.12.1:3333 |
| MQTT Topic | CustomByteBlock |

### 接收端参数 (`receiver/config/receiver_params.yaml`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| broker_host | 192.168.12.1 | MQTT Broker IP |
| broker_port | 3333 | MQTT Broker 端口 |
| topic | CustomByteBlock | MQTT 订阅 topic |
| display | true | 是否显示画面 |
| display_scale | 2 | 显示缩放倍数 |
| crosshair_offset_x | 0 | 准心 X 偏移 |
| crosshair_offset_y | 0 | 准心 Y 偏移 |

---

## 8. 参考

- 原项目: [Pacific_doorlock_sniper/](Pacific_doorlock_sniper/)
- 原编码器: `Pacific_doorlock_sniper/src/doorlock_sniper/src/video_encoder_node.cpp` — 图像预处理管线 + GStreamer H264编码 + 滑动窗口流控
- 原解码器: `Pacific_doorlock_sniper/src/doorlock_decoder/doorlock_decoder/video_decoder_node.py` — PyAV H264解码 + OpenCV显示
- 协议文档: `docs/RM2026通信协议_V2.0.0.pdf` — 串口协议第1章，自定义客户端协议第2章，CRC附录一
