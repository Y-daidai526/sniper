# AGENTS.md — 低带宽图传适配 RoboMaster 裁判系统

## 项目目标

将 [Pacific_doorlock_sniper](Pacific_doorlock_sniper/) 的低带宽视频压缩数据流适配到 **RoboMaster 裁判系统传输协议**，替换原有的 ROS2 Topic 通信层。

- **原有通信**：编码器 ←ROS2 VideoPacket→ 解码器
- **目标通信**：编码器 → 裁判系统串口协议 → 接收端 → 解码器
- **约束**：不动开源工程 `Pacific_doorlock_sniper/` 的代码，在根目录新建 `sender/` 和 `receiver/` 目录进行开发，参考和复制必要的代码。

---

## 开源基础：Pacific_doorlock_sniper 分析

### 项目来源

- RM2026 部署模式低带宽落点图传，五大湖联合大学
- 基于 ROS2 Kilted，海康机器人相机 (MVS SDK)
- 目标码率 ~40-80 kbps（~5-10 kB/s），压缩比极高
- 演示视频: <https://www.bilibili.com/video/BV12aDMBcEob>
- 开源报告: <https://bbs.robomaster.com/article/1883295>

### 源码目录结构

```
Pacific_doorlock_sniper/src/
├── hik_camera/              # 海康相机驱动 (C++ Composable Node)
│   ├── src/hik_camera_node.cpp
│   └── config/camera_6mm_MV-CS016-10UC.yaml
├── doorlock_sniper/         # 视频编码 + 分包 (C++ Composable Node)
│   ├── src/video_encoder_node.cpp
│   ├── include/doorlock_sniper/video_encoder_node.hpp
│   └── msg/VideoPacket.msg  # ROS2 自定义消息定义
├── doorlock_decoder/        # H264 解码 + 显示 (Python Node)
│   └── doorlock_decoder/video_decoder_node.py
└── bringup/                 # 启动文件
    └── launch/sniper.launch.py
```

### 系统拓扑

```
┌─ ComposableNodeContainer (同进程零拷贝) ─────────────────────────┐
│                                                                    │
│  hik_camera (C++)               VideoEncoderNode (C++)            │
│  ┌──────────────────┐           ┌──────────────────────────────┐  │
│  │ Bayer→BGR8       │ /image_raw│ centerCrop(800px)            │  │
│  │ 1440×1080@250fps │──────────→│ resize→300×300 (or 400×400) │  │
│  │ MVS SDK          │           │ 静态背景简化 + 运动拖影      │  │
│  └──────────────────┘           │ GStreamer x264enc→h264parse  │  │
│                                  │ 150B/300B 分包 → VideoPacket │  │
│                                  │ PUB: /video_stream           │  │
│                                  └──────────────┬───────────────┘  │
└─────────────────────────────────────────────────┼──────────────────┘
                                                  │ ROS2 Topic
                                                  │ RELIABLE, KeepLast(3000)
┌─────────────────────────────────────────────────┼──────────────────┐
│  独立进程                                       ▼                  │
│  VideoDecoderNode (Python)                                          │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │ SUB: /video_stream → seq_id排序 → PyAV H264解码 → 显示+准心 │  │
│  └──────────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────────┘
```

---

## 编码器详细分析

### 1. 图像预处理管线 (`preprocess_image()`)

| 步骤 | 操作 | 参数 |
|------|------|------|
| 1 | 中心裁剪 | crop_size=800px |
| 2 | 缩放 | 800² → output_size² (300 or 400) |
| 3 | 可选强制灰度 | force_monochrome=false |
| 4 | 背景模型更新 | `accumulateWeighted`, α=0.01 |
| 5 | 运动检测 | 差分→阈值14→腐蚀(2px)→膨胀(6px) |
| 6 | 静态区模糊 | GaussianBlur σ=1.8 |
| 7 | 中心保护区 | 150×150px 不做静态模糊 |
| 8 | 运动拖影 | 历史90帧 max 叠加到运动区 |

### 2. GStreamer H264 编码管线 (`initialize_gstreamer()`)

```
appsrc(BGR, 300×300@60fps)
  → videoconvert
  → x264enc (byte-stream, Annex-B)
  → h264parse (config-interval=-1, 每帧插入SPS/PPS)
  → appsink (caps: video/x-h264, byte-stream, au)
```

**低码率模式 (≤80 kbps)**:
- speed-preset: veryslow
- bframes=4, ref=5, rc-lookahead=40
- vbv-buf-capacity=500
- 特殊 option-string: `scenecut=0:aq-mode=2:mbtree=1:force-cfr=1`

**正常模式 (>80 kbps)**:
- tune: zerolatency
- bframes=0, rc-lookahead=0, sync-lookahead=0
- sliced-threads=true
- key-int-max: 2×fps

**输出**: Annex-B byte-stream (`00 00 00 01` 起始码分隔 NAL 单元)，SPS/PPS 每帧重复

### 3. 分包与流控 (`pull_stream_and_packetize()`)

```
appsink 拉取 H264 字节
    → 追加到 stream_buffer_ (vector<uint8_t>)
    → 以 150B(或300B) 为单位切分
    → 填充到 VideoPacket.data[150]
    → sequence_id++ 递增
    → 发布到 /video_stream
```

**滑动窗口限速**:
- 窗口时长: `bandwidth_window_s` = 2.0s
- 窗口字节上限: `bandwidth_limit_kbytes × 1000 × 2.0`
- 超过上限则暂缓发送，等待窗口滑动

**积压丢弃**:
- 积压上限: `bandwidth_limit_kbytes × 1000 × max_tx_delay_s` (1.0s)
- 超限时丢弃旧数据，对齐到 Annex-B 起始码边界 (减少解码崩溃)

**遥测**: 每秒输出一次 TX stats (窗口用量、积压、丢弃字节数)

---

## 解码器详细分析

### 1. 接收与组装 (`_packet_callback()`)

```python
def _packet_callback(self, msg):
    # 丢包检测：seq_id 跳变 → 重置解码器
    if msg.sequence_id != self.last_seq + 1:
        self._reset_decoder(reason='sequence gap')

    chunk = bytes(msg.data)  # 150B 原始字节

    # PyAV H264 解码管线
    parsed = self.codec.parse(chunk)       # Annex-B → NAL 单元
    for packet in parsed:
        for frame in self.codec.decode(packet):   # NAL → 图像帧
            self._handle_decoded_frame(frame)
```

### 2. 解码器重置 (`_reset_decoder()`)

- 重新创建 `av.CodecContext.create('h264', 'r')`
- 设置 `LOW_DELAY` flag
- 原因: 任意 150B 分片丢失都会破坏码流同步，只能等待下一组 SPS/PPS + IDR

### 3. 显示 (`_display_loop()`)

- 独立线程从 `frame_queue` (maxsize=3) 取帧
- `cv2.imshow` + 准心叠加（淡紫色十字 + 绿色中心点）

---

## VideoPacket 消息定义

```ros2
# doorlock_sniper/msg/VideoPacket.msg
uint64 sequence_id       # 全局包序号（用于丢包检测）
uint64 timestamp_ns      # ROS2 时间戳（用于延时测量）
uint8[150] data          # 固定150字节 H264 Annex-B 字节流
                         # 最后一包不足则补零
```

**硬约束**: `VideoPacket.data` 长度固定为 150 字节（由 .msg 定义确定），编码器强制 `packet_size = 150`。

---

## 编码器↔解码器通信总结

| 层面 | 实现 |
|------|------|
| **传输层** | ROS2 Topic `/video_stream`, RELIABLE QoS, KeepLast(3000) |
| **消息封装** | `VideoPacket`: seq_id(8B) + ts_ns(8B) + data(150B) = 166B/包 |
| **载荷编码** | H264 Annex-B byte-stream (NAL 单元以 `00 00 00 01` 分隔) |
| **流控机制** | 发送端滑动窗口限速 + 积压超限丢弃（对齐起始码） |
| **错误恢复** | 编码器每帧重复 SPS/PPS；解码器遇 gap 硬重置等 IDR |
| **带宽设计** | ~7 kB/s 典型值，对应 ~56 kbps 视频码率 |

---

## 适配 RoboMaster 裁判系统的计划

### 已有认知

1. 裁判系统通过 **串口/UART** 与机器人通信，带宽极低（通常 ≤10 kB/s）
2. 通信协议为 **固定帧格式**：帧头 + 长度 + 命令码 + 数据段 + CRC 校验
3. 每个数据帧有效载荷有限（通常 ≤128 字节），需要将 150B VideoPacket 拆分或重组
4. 串口传输是**不可靠**的（无 ROS2 RELIABLE 那样的重传机制），需要自行处理丢包

### 待确认问题

1. **裁判系统通信协议的具体帧格式**——帧头、命令码、最大数据段长度、CRC 算法？
2. **硬件拓扑**——发送端设备（迷你PC？妙算？）和接收端（操作手站？）
3. **串口参数**——波特率？
4. **是否有现成的裁判系统通信 SDK 或示例？**

### 规划的新目录结构 (待实现)

```
/home/y/sniper/
├── AGENTS.md                            # 本文件
├── Pacific_doorlock_sniper/             # 开源代码 (不动)
├── sender/                              # 发送端 (新建)
│   ├── 从 doorlock_sniper 参考:
│   │   ├── video_encoder_node.cpp/hpp   # 图像预处理 + GStreamer 编码 + 分包
│   │   └── VideoPacket.msg              # 消息格式定义
│   └── 新增:
│       ├── referee_protocol.cpp/hpp     # 裁判系统协议封装（替代 ROS2 Publisher）
│       └── sender_main.cpp              # 发送端入口
├── receiver/                            # 接收端 (新建)
│   ├── 从 doorlock_decoder 参考:
│   │   └── video_decoder_node.py        # H264 解码 + 显示
│   └── 新增:
│       ├── referee_protocol.py          # 裁判系统协议解析（替代 ROS2 Subscriber）
│       └── receiver_main.py             # 接收端入口
└── common/                              # 共享定义 (新建)
    └── protocol_defs.h / .py            # 裁判系统帧格式常量
```

### 关键适配点

1. **传输层替换**: ROS2 Topic → 裁判系统串口帧
   - VideoPacket 的 150B data 需要映射到裁判系统帧的数据段
   - 如果裁判帧数据段 <150B，需要二次分包；如果 >150B，可以合并多包

2. **序列号**: 保留 `sequence_id` 用于丢包检测

3. **流控**: 发送端滑动窗口限速逻辑可复用，但流控触发从"暂停 publish"变为"暂停串口写入"

4. **错误恢复**: 解码器的 gap 检测 + 重置逻辑保持不变

5. **时间戳**: `timestamp_ns` 需要调整为裁判系统的时间基准（或保留本地时间戳用于延时统计）

---

## 关键参数速查

| 参数 | 默认值 | 说明 |
|------|--------|------|
| crop_size | 800 | 中心裁剪尺寸 (px) |
| output_size | 300 | 编码分辨率 (px²) |
| output_fps | 60 | 编码帧率 |
| target_bitrate | 40-80 | x264 目标码率 (kbps) |
| packet_size | 150 | 固定分包大小 (byte) |
| bandwidth_limit_kbytes | 7.0 | 发送带宽上限 (kB/s) |
| bandwidth_window_s | 2.0 | 限速滑动窗口 (s) |
| max_tx_delay_s | 1.0 | 最大发送队列时延 (s) |
| motion_trail_frames | 90 | 拖影历史帧数 |
| bg_blur_sigma | 1.8 | 静态区高斯模糊 σ |
| center_clear_size | 150 | 中心保护区 (px) |
