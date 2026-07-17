# RoboMaster 0x0310 Sniper 项目说明

这个文件是给后续 AI 或开发者看的项目交接文档。内容必须与当前代码一致，不记录旧计划、旧目录或已经删除的实现。

## 当前项目形态

工作区的有效工程：

```text
sender/                 机器人端，ROS 2 C++ 与 Python 节点
receiver/               操作手端，ROS 2 Python 节点
docs/                   RoboMaster 通信协议 PDF
tools/mqtt_test.py      独立 MQTT inner-seq 诊断工具
start_sender.sh         sender 构建和启动入口
start_receiver.sh       receiver 构建和启动入口
```

节点源码直接放在与可执行名一致的目录中：

```text
sender/
  camera_encoder_node/
  hik_camera_node/
  video_encoder_node/
  rm_frame_encoder_node/
  serial_sender_node/
  mqtt_broker_node/
  mqtt_sender_node/

receiver/
  mqtt_receiver_node/
  video_decoder_node/
```

节点文件使用 `*_node` 命名。`rm_frame.*`、`serial_writer.*`、MQTT helper、参数 helper 和 protobuf 等工具文件不强制带 `_node`。

## 运行链路

sender launch 启动 5 个进程、6 个 ROS 节点：

```text
camera_encoder_node process
  /sender/hik_camera
    -> /sender/image_raw                 sensor_msgs/Image, bgr8
    -> /sender/camera_info               sensor_msgs/CameraInfo
  /sender/video_encoder
    -> /sender/encoded_stream            std_msgs/UInt8MultiArray, H264 Annex-B

/sender/rm_frame_encoder
  -> /sender/rm_frame                    std_msgs/UInt8MultiArray, 完整 309B 帧

/sender/serial_sender
  -> USB 串口

/sender/mqtt_broker
  -> 可选 amqtt broker

/sender/mqtt_sender
  -> MQTT CustomByteBlock protobuf
```

`camera_encoder_node` 是进程入口，不是额外的 ROS 节点。它创建 `HikCameraNode` 和 `VideoEncoderNode`，用同一个单线程 executor 运行，并启用 ROS 2 intra-process communication。不要为了“显示一个主节点”再创建空的 `/sender/camera_encoder`。

receiver launch 启动 2 个进程、2 个 ROS 节点：

```text
/receiver/mqtt_receiver
  -> /receiver/encoded_stream            UInt8MultiArray, 每条 299B

/receiver/video_decoder
  -> /receiver/video_stream/raw          Image, 原始解码 BGR8
  -> /receiver/video_stream              Image, 缩放并绘制准星
  -> OpenCV 窗口                         与 video_stream 内容一致
```

项目业务 topic 都位于 `/sender` 或 `/receiver` namespace。`/rosout`、`/parameter_events` 等 ROS 全局基础 topic 不在这个约束内。外部 MQTT topic `CustomByteBlock` 也不受 ROS namespace 影响。

## 构建和运行

推荐从仓库根目录运行：

```bash
./start_sender.sh
./start_receiver.sh client_id:=1
```

两个脚本都使用仓库根目录的 `build/`、`install/`、`log/`，分别只构建自己的包，并把额外参数原样传给 `ros2 launch`。

手动运行：

```bash
colcon build --symlink-install --packages-select sender receiver
source install/setup.bash
ros2 launch sender sender.launch.py
ros2 launch receiver receiver.launch.py client_id:=1
```

launch 文件位于标准包目录：

```text
sender/launch/sender.launch.py
receiver/launch/receiver.launch.py
```

两个 launch 都设置 `RCUTILS_CONSOLE_OUTPUT_FORMAT` 去掉 ROS 日志时间戳，打印实际配置路径，并在任一业务进程退出后关闭同一端的其他进程。

配置查找顺序：

```text
在包目录运行：      config/*_params.yaml
在仓库根目录运行：  sender/config/sender_params.yaml
                    receiver/config/receiver_params.yaml
源码配置不存在：    install share 中的 config
```

receiver 的 MQTT `client_id` 是 launch 参数，不写入 YAML。未提供时 launch 通过 `input()` 询问；空输入、EOF 或非交互环境必须显式传入。

## 配置原则

YAML 是用户可调业务参数的唯一入口。launch 不复制业务默认值，普通节点的 required parameter 也不在代码中隐藏另一套业务值。海康节点保留 SDK 参数默认声明方式，是明确例外。

配置文件：

```text
sender/config/sender_params.yaml
receiver/config/receiver_params.yaml
```

YAML 使用完整节点路径，例如：

```text
/sender/hik_camera
/sender/video_encoder
/sender/serial_sender
/receiver/mqtt_receiver
/receiver/video_decoder
```

### Sender 参数

`/sender/hik_camera`：

```text
use_sensor_data_qos
camera_name
camera_info_url
exposure_time
gain
frame_rate
```

`frame_rate` 是整个相机编码链唯一的帧率配置。海康节点设置设备帧率，`camera_encoder_node` 再把同一个值作为构造参数传给编码器。编码器不声明帧率 ROS 参数，也不在图像回调中二次限帧。该参数仅支持启动时设置。

`/sender/video_encoder`：

```text
crop_size
output_size
force_monochrome
static_simplify
motion_threshold
motion_erode_px
motion_dilate_px
motion_trail_frames
trail_disable_motion_ratio
bg_update_alpha
bg_blur_sigma
center_clear_size
target_bitrate
x264_preset
enable_display
```

编码器固定订阅同 namespace 的 `image_raw`，不提供 `input_topic` 参数。

`/sender/serial_sender`：

```text
enabled
max_rate_hz
max_backlog_frames
```

`/sender/mqtt_sender`：

```text
enabled
host
port
topic
```

`/sender/mqtt_broker`：

```text
enabled
host
port
```

`rm_frame_encoder` 没有参数。配置关闭 serial/MQTT 节点时，节点仍由 launch 创建并保持空闲。

### Receiver 参数

`/receiver/mqtt_receiver`：

```text
broker_host
broker_port
topic
```

`/receiver/video_decoder`：

```text
queue_size
display
display_scale
crosshair_offset_x
crosshair_offset_y
crosshair_width
```

`display_scale` 同时决定 `/receiver/video_stream` 的尺寸和本地窗口尺寸。`display=false` 只关闭窗口，不停止两个图像 topic。

## Sender 包

`sender` 使用 `ament_cmake` 和 `ament_cmake_python`。主要文件：

```text
sender/CMakeLists.txt
sender/package.xml
sender/launch/sender.launch.py
sender/config/sender_params.yaml
sender/camera_encoder_node/camera_encoder_node.cpp
sender/hik_camera_node/hik_camera_node.*
sender/video_encoder_node/video_encoder_node.*
sender/rm_frame_encoder_node/rm_frame_encoder_node.*
sender/rm_frame_encoder_node/rm_frame.*
sender/serial_sender_node/serial_sender_node.*
sender/serial_sender_node/serial_writer.*
sender/mqtt_broker_node/
sender/mqtt_sender_node/
```

安装后的可执行入口：

```text
install/sender/lib/sender/camera_encoder_node
install/sender/lib/sender/rm_frame_encoder_node
install/sender/lib/sender/serial_sender_node
install/sender/lib/sender/mqtt_broker_node
install/sender/lib/sender/mqtt_sender_node
```

`hik_camera` 和 `video_encoder` 没有独立可执行入口，只作为 `camera_encoder_node` 进程里的两个 ROS 节点存在。

### 海康相机

海康节点依赖：

```text
/opt/MVS/include
/opt/MVS/lib/64
```

主要行为：

```text
枚举 USB 海康相机并打开第一个设备
读取 Width/Height
设置曝光、增益和设备采集帧率
MV_CC_GetImageBuffer 获取 Bayer 图像
cv::COLOR_BayerRG2RGB 转到复用的 bgr8 Image buffer
发布 image_raw 和 CameraInfo
取图失败时 StopGrabbing/StartGrabbing
连续失败超过 5 次后 shutdown
```

帧率通过以下 MVS 属性设置：

```text
AcquisitionFrameRateEnable = true
AcquisitionFrameRate = frame_rate
```

设置失败时构造直接报错，不允许相机与编码器使用不同帧率。曝光时间超过帧周期时，物理实际帧率可能低于配置值。

保留的 SDK 经验值：

```text
ColorTransformationEnable = true
Gamma = 7.5
CCMEnable = false
```

`exposure_time` 和 `gain` 可运行时更新，`frame_rate` 只在启动时读取。图像发布和编码订阅都使用深度 1；默认是 sensor-data Best Effort QoS。

相机仍以 const-reference 发布复用消息。intra-process 消除了全尺寸图像的 DDS 序列化和跨进程复制，但通常仍有一次进程内消息复制，不是完全零拷贝。

### 视频编码器

`VideoEncoderNode` 的构造函数接收海康节点已经设置成功的帧率，用于 GStreamer raw caps 和关键帧间隔，不做回调级 FPS gate。

预处理：

```text
中心裁剪 crop_size
resize 到 output_size x output_size
可选灰度化
可选静态背景简化：
  背景模型和 absdiff
  threshold + erode/dilate
  中心区域强制清晰
  静态区域模糊
  运动区域和可选 motion trail 保留细节
```

GStreamer：

```text
appsrc -> videoconvert -> x264enc -> h264parse -> appsink
```

pipeline 元素创建、link 或进入 PLAYING 失败时构造函数抛异常。appsink 输出 H264 Annex-B access unit，节点复制到 `UInt8MultiArray` 并可靠发布 `/sender/encoded_stream`，publisher depth 为 20。

`target_bitrate <= 80` 使用低码率质量模式，保留 B 帧、lookahead、AQ 和 mbtree；更高码率使用无 B 帧、无 lookahead 的低延迟模式。

`enable_display=true` 时创建 sender 四宫格预览，16ms timer 与编码回调运行在同一个单线程 executor，避免 Qt 跨线程创建/销毁。GUI 很慢时仍可能影响编码时序；生产环境不需要预览时可在 YAML 关闭。

### RM 帧编码

`RmFrameEncoderNode` 订阅 `/sender/encoded_stream`，把每条 H264 输出追加到连续 buffer。每够 299B 就顺序取出一段：

```text
data[0]      inner seq，uint8 回绕
data[1:300]  299B H264 Annex-B 字节
```

随后调用 `build_rm_frame()` 添加裁判帧 seq、帧头和 CRC，发布完整 `/sender/rm_frame`。不补零、不重复旧数据、不按 access unit 重新切包。

C++ 的帧常量、构帧、CRC8 和 CRC16 全部集中在：

```text
sender/rm_frame_encoder_node/rm_frame.hpp
sender/rm_frame_encoder_node/rm_frame.cpp
```

头文件只公开跨节点需要的常量、`RmFrame` 和 `build_rm_frame()`；CRC 参数和函数位于 `.cpp` 内部。

### 串口发送

`SerialSenderNode` 只接收完整 309B `/sender/rm_frame`。消息长度不正确时丢弃；合法消息进入完整帧队列。

队列达到 `max_backlog_frames` 时清空整个队列，包括刚加入的新帧，并把 `drops` 加一。独立线程按 `max_rate_hz` 取帧，最大允许 50Hz。

一次发送尝试：

```text
调用 SerialWriter::write_frame()
串口已打开：完整 write 后等待 tcdrain
串口未打开或功能 disabled：立即返回
函数返回后累计 packets
```

所以 stats 的包速率是发送线程完成尝试的速率，即使串口断开也会继续统计。

串口扫描：

```text
/dev/ttyACM*
/dev/ttyUSB*
```

固定参数：921600 baud、8N1、无硬件流控、raw mode。热插拔线程每 500ms 检查，打开失败或写错误后会继续重试。

约每秒日志：

```text
[serial stats]: rate=...pkt/s data=...kB/s packets=... drops=... backlog=...frames port=...
```

`data` 按 `rate * 299B / 1000` 计算。`port` 是设备路径、`disconnected` 或 `disabled`。

### Sender MQTT

两个 Python 包分别安装：

```text
sender/mqtt_broker_node/
sender/mqtt_sender_node/
```

`MqttBrokerNode` 可启动独立 amqtt broker。`MqttSenderNode` 订阅 `/sender/rm_frame`，用 `mqtt_sender_node/rm_frame.py` 校验长度、SOF、data_len、CRC8、cmd_id、CRC16，然后发布 `frame[7:307]`。

protobuf schema：

```proto
message CustomByteBlock {
  optional bytes data = 1;
}
```

构造消息时必须先创建对象再赋值：

```python
message = CustomByteBlock_pb2.CustomByteBlock()
message.data = data_300
```

当前生成的 protobuf 类不接受 `CustomByteBlock(data=...)` 关键字构造。

Paho 固定使用 2.1.0 callback API v2。初次连接失败由 ROS timer 重试；已启动的网络 loop 由 Paho 负责重连。日志只保留状态和节流后的异常。

## 0x0310 帧格式

固定 309B：

```text
offset  size  field
0       1     SOF       = 0xA5
1       2     data_len  = 300 / 0x012C, little-endian
3       1     seq       = 裁判帧 seq，uint8 wrap
4       1     CRC8      = CRC8(frame[0:4])
5       2     cmd_id    = 0x0310, little-endian
7       300   data      = 1B inner seq + 299B H264
307     2     CRC16     = CRC16(frame[0:307]), little-endian
```

CRC 模型：

```text
CRC8  normal polynomial 0x31, reflected 0x8C, init 0xFF
CRC16 normal polynomial 0x1021, reflected 0x8408, init 0xFFFF
算法均为 reflected right shift
```

C++ 构帧与 Python MQTT parser 必须保持一致：

```text
sender/rm_frame_encoder_node/rm_frame.cpp
sender/mqtt_sender_node/rm_frame.py
```

协议来源是 `docs/` 下 RoboMaster 2026 通信协议 PDF。

## Receiver 包

`receiver` 使用 `ament_python`。主要文件：

```text
receiver/setup.py
receiver/package.xml
receiver/launch/receiver.launch.py
receiver/config/receiver_params.yaml
receiver/mqtt_receiver_node/
receiver/video_decoder_node/
```

console scripts：

```text
mqtt_receiver_node
video_decoder_node
```

### MQTT 接收

`MqttReceiverNode` 使用 Paho callback API v2。启动连接失败不会退出；timer 重试初始连接，断开 warning 约 3 秒节流。连接成功后以 MQTT QoS 0 订阅配置 topic。

每条 payload：

```text
解析 CustomByteBlock
要求 data 字段存在且长度为 300
data[0] 做 inner seq 统计
重复 seq 丢弃
gap 记录并打印 missing 数
data[1:300] 发布 /receiver/encoded_stream
```

seq 规则：

```text
diff = (seq - last_seq) & 0xFF
diff == 0  duplicate
diff > 1   gap = diff - 1
```

约每秒日志：

```text
rate=...pkt/s data=...kB/s packets=... gaps=... dups=...
```

### 解码和图像输出

`VideoDecoderNode` 订阅 299B `/receiver/encoded_stream`，使用 PyAV H264 parser/decoder。`queue_size` 是 encoded subscription depth，不能把它改成图像用的 depth 1，否则任意丢失 H264 分片会破坏码流。

PyAV 全局日志级别设为 PANIC，并在创建、parse、decode 时使用 `av.logging.Capture(local=False)`，防止 FFmpeg/PyAV 内部日志泄漏。异常只打印一行：

```text
decode error count=N: <single-line detail>
```

每个解码帧转换为 BGR8，然后：

```text
/receiver/video_stream/raw
  原始解码尺寸，不含准星

/receiver/video_stream
  按 display_scale 放大
  使用 crosshair_offset_x/y 和 crosshair_width 绘制准星
```

两个 `sensor_msgs/Image` 使用同一 ROS 时间戳、正确的 width/height/step，publisher 均为 Best Effort、Volatile、Keep Last 1。

`display=true` 时窗口显示处理后图像。显示队列只保留最新 1 帧，队列满时先丢旧帧再加入新帧。`cv2.waitKey(1)` 不处理 `q`，窗口按键不会关闭 ROS。

销毁节点时停止显示线程；`destroy_node()` 可重复调用，`main()` 最终使用 `rclpy.try_shutdown()`。

sender 和 receiver 的 `.proto` 与生成的 `_pb2.py` 必须同步修改。

## 依赖策略

系统依赖写在 README。Python 图像依赖优先使用 apt：

```text
python3-av
python3-opencv
```

MQTT 客户端固定 `paho-mqtt 2.1.0`，通过 pip 安装；`amqtt` 只用于 sender 内置 broker。不要新增 `requirements.txt`，除非项目依赖策略明确改变。

## README 维护规则

README 面向使用者，只写依赖、配置路径、启动命令和公开图像 topic，不展开参数逐项解释或内部架构争论。实现细节放在本文件。

不要修改未跟踪的：

```text
SENDER_CODE_READING.md
RECEIVER_CODE_WALKTHROUGH.md
```

它们不是当前架构的权威说明。

## 改动检查

按影响范围执行最小检查：

```bash
python3 -m py_compile \
  sender/mqtt_broker_node/mqtt_broker_node.py \
  sender/mqtt_broker_node/broker.py \
  sender/mqtt_sender_node/mqtt_sender_node.py \
  sender/mqtt_sender_node/publisher.py \
  sender/mqtt_sender_node/rm_frame.py \
  receiver/mqtt_receiver_node/mqtt_receiver_node.py \
  receiver/video_decoder_node/video_decoder_node.py

colcon build --symlink-install --packages-select sender
colcon build --symlink-install --packages-select receiver
git diff --check
```

协议改动必须测试合法 309B 帧，以及错误长度、SOF、CRC8、cmd_id、CRC16 的拒绝。

真机 sender 检查：

```text
海康相机成功枚举
/sender/image_raw 帧率接近 frame_rate
/dev/ttyUSB* 或 /dev/ttyACM* 成功打开
MQTT broker 和 publisher 连接
serial stats 持续输出
Ctrl-C 无 camera -11、Qt 跨线程 warning 或残留进程
```

receiver 检查：

```text
/receiver/encoded_stream 为 UInt8MultiArray
/receiver/video_stream/raw 为原始 Image
/receiver/video_stream 为缩放加准星 Image
两图时间戳相同
display=false 时仍发布两图
解码错误保持单行且无 PyAV 内部日志
```
