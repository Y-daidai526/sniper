# RoboMaster 0x0310 Sniper 项目说明

这个文件是给后续 AI 或开发者看的项目交接文档。内容必须跟当前代码一致，不写旧计划，不写已经删掉的方案。

## 当前项目形态

当前工作区的有效工程是两个新包：

```text
sender/                  机器人端，ROS2 C++
receiver/                操作手端，ROS2 Python
Pacific_doorlock_sniper/ 参考工程和可选旧解码器验证来源
docs/                    RoboMaster 通信协议 PDF
start.sh                 单端构建并运行脚本
```

`Pacific_doorlock_sniper/` 只作为参考来源和旧解码器验证来源。当前主链路由 `sender/` 和 `receiver/` 完成。

主链路：

```text
sender
  Hik camera
    -> /image_raw
    -> GStreamer x264 Annex-B H264
    -> 299B H264 + 1B inner seq
    -> 300B 0x0310 data
    -> 309B RoboMaster 裁判系统串口帧
    -> USB 串口
    -> 可选 local_test MQTT

receiver
  MQTT CustomByteBlock protobuf
    -> 300B data
    -> data[0] 做 seq 统计
    -> data[1:300] 喂给 PyAV H264 parser/decoder
    -> OpenCV 显示
```

可选兼容调试链路：

```text
sender
  同一份 H264 Annex-B 输出
    -> 150B 分包
    -> 手写 CDR，类型名 doorlock_sniper/msg/VideoPacket
    -> ROS2 GenericPublisher 发布 /video_stream
    -> Pacific 旧 decoder 验证画面
```

## 构建和运行

推荐入口：

```bash
./start.sh sender
./start.sh receiver
```

`./start.sh sender` 只构建 `sender`，source 根目录 `install/setup.bash`，然后运行 `ros2 launch sender sender.launch.py`。

`./start.sh receiver` 只构建 `receiver`，source 根目录 `install/setup.bash`，然后运行 `ros2 launch receiver receiver.launch.py`。

手动运行：

```bash
colcon build --packages-select sender receiver
source install/setup.bash
ros2 launch sender sender.launch.py
ros2 launch receiver receiver.launch.py
```

launch 文件只做两件事：

```text
1. 设置 RCUTILS_CONSOLE_OUTPUT_FORMAT，去掉 ROS 日志时间戳。
2. 选择并打印实际使用的 config 文件路径。
```

launch 不写业务参数默认值，不覆盖业务参数。

config 文件查找规则：

```text
优先读取当前工作目录下的源码配置：
  sender/config/sender_params.yaml
  receiver/config/receiver_params.yaml

如果源码配置不存在，再退回 install share 配置：
  install/*/share/*/config/*.yaml
```

这样在仓库根目录运行时，改源码 config 就是改实际运行配置。启动日志会打印 `sender config: ...` 或 `receiver config: ...`。

## 配置原则

config 是唯一可配置入口。用户能调的东西写在 YAML；不希望用户调的东西直接写在底层代码使用位置。

不要出现这种状态：

```text
YAML 写一份默认值
launch 又写一份默认值
代码 declare_parameter 又写一份默认值
```

当前要求：

```text
launch 不放业务参数
README 不展开参数说明，只告诉配置文件路径
AGENTS.md 说明参数契约和实现细节
只用一两次、没有协议含义的常量直接写在调用位置
协议字段、CRC 模型、跨 C++/Python 必须一致的数字可以保留命名常量
```

示例：

```text
receiver stats 的 1 秒窗口直接写在 StatsTracker.record() 判断处。
MQTT 接收端校验 300B data 时直接写 300。
local_test 从 stdin 读取 309B 帧时直接写 309。
```

保留命名常量的例子：

```text
0x0310 帧格式常量
CRC8/CRC16 多项式
H264 主链路 299B 分片
/video_stream 150B 兼容分片
```

## 依赖策略

系统依赖写在 `README.md`。Python 依赖优先用 apt：

```text
python3-av
python3-opencv
```

MQTT 客户端固定使用 `paho-mqtt 2.1.0` 和 callback API v2，通过 pip 安装。`amqtt` 只用于 sender local_test 内置 MQTT broker。README 里记录了 Debian/Ubuntu externally-managed Python 的处理方式。

不要新增 `requirements.txt`，除非项目策略明确改变。

## sender 包

`sender` 是 C++ ROS2 包，使用 `ament_cmake`。

重要文件：

```text
sender/main.cpp                         进程入口和节点 wiring
sender/CMakeLists.txt                   构建目标和安装规则
sender/launch/sender.launch.py          launch 入口
sender/config/sender_params.yaml        sender 唯一配置文件
sender/camera/hik_camera_node.*         海康 MVS 相机节点
sender/encoder/video_encoder_node.*     预处理、x264、分包、统计
sender/protocol/serial_frame.*          0x0310 构帧
sender/protocol/crc8.hpp                CRC8
sender/protocol/crc16.hpp               CRC16
sender/serial/serial_writer.*           USB 串口热插拔和写帧
sender/serial/serial_send_worker.*      H264 buffer、独立发送线程、分包和统计
sender/local_test/local_test_bridge.*   fork/exec Python local_test
sender/local_test/main.py               stdin 309B frame -> MQTT protobuf
sender/local_test/serial_parser.py      Python 侧 0x0310 校验
sender/local_test/mqtt_server.py        amqtt broker 和 paho publisher
sender/local_test/proto/                CustomByteBlock protobuf
```

安装后目标：

```text
install/sender/lib/sender/sender_node
install/sender/share/sender/config/
install/sender/share/sender/launch/
install/sender/share/sender/local_test/
```

### sender 进程结构

`sender/main.cpp` 在一个进程里创建：

```text
sender_runtime
hik_camera
video_encoder
```

同时持有：

```text
sniper::serial::SerialSendWorker
```

`video_encoder` 通过 `set_serial_stream_callback()` 把 appsink 产出的 H264 Annex-B 字节交给 `SerialSendWorker`。编码回调只复制字节，不执行分包、CRC 或 I/O。

`SerialSendWorker` 启动独立 C++ 发送线程，持有：

```text
SerialWriter：串口热插拔和完整帧写入
LocalTestBridge：local_test 子进程和 stdin 写入
```

发送线程按 `serial_max_rate_hz` 独立调度，从 buffer 取 299B H264，加 inner seq 并调用 `sniper::protocol::build_frame()` 构造完整 309B 0x0310 帧。编码器与发送线程之间只有一把 buffer mutex 和一个 condition variable，CRC、串口、local_test 和日志均不持有该锁。

local_test 脚本目录不进 config。代码按安装目录查找：

```text
share/sender/local_test
```

开发环境查不到安装目录时，退回：

```text
<当前工作目录>/sender/local_test
```

### sender 配置

配置文件：

```text
sender/config/sender_params.yaml
```

`sender_runtime` 当前读取：

```text
enable_serial
enable_local_test
local_test_start_broker
local_test_mqtt_host
local_test_mqtt_port
local_test_mqtt_topic
```

`video_encoder` 当前读取：

```text
input_topic
video_stream_topic
crop_size
output_size
output_fps
target_bitrate
x264_preset
enable_video_stream
static_simplify
motion_threshold
motion_erode_px
motion_dilate_px
motion_trail_frames
trail_disable_motion_ratio
bg_update_alpha
bg_blur_sigma
center_clear_size
force_monochrome
bandwidth_limit_kbytes
bandwidth_window_s
serial_max_rate_hz
max_tx_delay_s
enable_display
```

`sender_runtime` 和 `video_encoder` 的配置参数应该由 YAML 提供。缺参数时应该尽早暴露，而不是悄悄使用另一处默认值。

### 海康相机节点

海康相机节点来自 Pacific 已验证行为。不要随手重构它，不要为了“统一参数风格”去改它。

当前相机节点文件：

```text
sender/camera/hik_camera_node.hpp
sender/camera/hik_camera_node.cpp
```

相机节点使用 Hik MVS SDK：

```text
/opt/MVS/include
/opt/MVS/lib/64
```

行为概要：

```text
枚举 USB 海康相机
打开第一个相机
读取 Width/Height
发布 image_transport camera publisher：image_raw
MV_CC_GetImageBuffer 获取图像
cv::COLOR_BayerRG2RGB 转换
发布 Image + CameraInfo
取图失败时 StopGrabbing/StartGrabbing 重启采集
连续失败超过 5 次后 shutdown
```

保留在代码里的 SDK 经验值：

```text
AcquisitionFrameRate = 250.000
ColorTransformationEnable = true
Gamma = 7.5
CCMEnable = false
```

### 编码器和分包

`VideoEncoderNode` 订阅 `input_topic`，默认配置为 `/image_raw`。

预处理流程：

```text
中心裁剪 crop_size
resize 到 output_size x output_size
可选灰度化
可选静态背景简化：
  建立灰度背景模型
  absdiff + threshold 得到运动 mask
  erode/dilate 去噪和保细节
  center_clear_size 中心区域强制清晰
  静态区域模糊
  运动区域保留原细节
  可选 motion trail
```

GStreamer pipeline：

```text
appsrc -> videoconvert -> x264enc -> h264parse -> appsink
```

低码率模式条件：

```text
target_bitrate <= 80
```

低码率模式使用 repeat headers、B 帧、lookahead、AQ、mbtree 等参数，目标是保住低带宽画面质量。

appsink 输出的 H264 Annex-B 字节进入两个独立 buffer：

```text
SerialSendWorker::stream_buffer_        0x0310 主链路
VideoEncoderNode::ros2_stream_buffer_   /video_stream 兼容调试链路
```

主链路分包：

```text
独立发送线程中 stream_buffer_ 至少有 299B，且 rate gate 允许时：
  data[0] = serial_inner_seq_++
  data[1:300] = 下一段 299B H264
  build_frame 后写 SerialWriter 和可选 LocalTestBridge
```

不补零，不重复旧包。H264 不足 299B 时发送线程等待新数据。每次最多发送一包，调度延迟后不突发补发。

`/video_stream` 兼容分包：

```text
ros2_stream_buffer_ 至少有 150B，且带宽窗口允许时：
  发布一个 VideoPacket-compatible CDR sample
```

主链路由独立发送线程按 `serial_max_rate_hz` 限速，最大 50Hz，不再依赖图像回调频率。`/video_stream` 由 `bandwidth_limit_kbytes` 和 `bandwidth_window_s` 控制，不反向影响主链路。

backlog 裁剪：

```text
serial max backlog = serial_max_rate_hz * 299B * max_tx_delay_s
topic max backlog  = bandwidth_limit_kbytes * 1000 * max_tx_delay_s
```

超过 backlog 后丢旧 H264，并尽量对齐到后续 Annex-B start code。

### /video_stream 兼容发布

用途：用 Pacific 旧解码器验证 sender 编码链路。

实现：

```text
rclcpp::GenericPublisher
topic = video_stream_topic
type  = doorlock_sniper/msg/VideoPacket
QoS   = KeepLast(3000).reliable()
```

手写 CDR 当前布局：

```text
offset  size  field
0       4     CDR encapsulation header: 00 01 00 00
4       8     sequence_id, uint64 little-endian
12      8     timestamp_ns, uint64 little-endian
20      150   payload bytes
```

`SerializedMessage` 分配容量是 172B，但实际 `buffer_length` 是写入长度 170B。

启用条件：

```text
enable_video_stream=true
运行环境能创建 doorlock_sniper/msg/VideoPacket type support
```

缺 type support 或发布失败时，只禁用 `/video_stream` 兼容发布，不影响 0x0310 串口和 local_test/MQTT 主链路。

### 串口

`SerialWriter` 扫描：

```text
/dev/ttyACM*
/dev/ttyUSB*
```

串口参数在底层代码固定：

```text
921600 baud
8 data bits
no parity
1 stop bit
no hardware flow control
raw mode
```

monitor loop 每 500ms 检查一次热插拔。无串口时编码继续跑，stats 里 `port=disconnected`。配置关闭串口时 `port=disabled`。

串口只写完整 309B frame，不写裸 300B data。

### local_test

`local_test` 是本地 MQTT 测试链路，不再使用旧 debug 命名。

C++：

```text
sender/local_test/local_test_bridge.hpp
sender/local_test/local_test_bridge.cpp
```

Python：

```text
sender/local_test/main.py
sender/local_test/serial_parser.py
sender/local_test/mqtt_server.py
```

流程：

```text
LocalTestBridge fork/exec Python
sender 把完整 309B 裁判帧写到子进程 stdin
main.py 每次 read(309)
serial_parser.py 校验 SOF、data_len、CRC8、cmd_id、CRC16
mqtt_server.py 发布 CustomByteBlock(data=frame[7:307])
```

`local_test_start_broker=true` 时，Python 使用 `amqtt` 启动内置 broker。启动失败时尝试连接同地址已有 broker。

local_test 日志只保留异常和状态：

```text
[mqtt] ...
[local_test] stdin EOF ...
[local_test] short read ...
[local_test] parse error ...
```

不要恢复周期性 published frame 计数日志。

### sender stats

sender 统计约 1 秒打印一次。serial stats 的间隔和计数在 `SerialSendWorker::run()`，topic stats 的间隔和计数在 `VideoEncoderNode::pull_stream_and_packetize()`。

串口 stats logger：

```text
serial stats
```

格式：

```text
[INFO] [serial stats]: rate=26.8pkt/s data=8.01kB/s packets=1234 drops=0 backlog=80B port=/dev/ttyACM0
```

字段：

```text
rate      当前窗口发包率
data      rate * 299B / 1000，只按 H264 有效载荷算
packets   累计 0x0310 data 包数
drops     累计 serial backlog 裁剪事件数
backlog   当前 SerialSendWorker::stream_buffer_ 字节数
port      当前串口路径、disconnected、disabled 或 unknown
```

topic stats logger：

```text
topic stats
```

格式：

```text
[INFO] [topic stats]: rate=90.0pkt/s data=13.50kB/s packets=4321 drops=0 backlog=120B
```

只有 `/video_stream` GenericPublisher 实际启用时才打印 `[topic stats]`。

sender stats 不打印 `errors` 字段。

## 0x0310 帧格式

固定 309B：

```text
offset  size  field
0       1     SOF       = 0xA5
1       2     data_len  = 300 / 0x012C, little-endian
3       1     seq       = 裁判系统帧 seq，uint8 wrap
4       1     CRC8      = CRC8(frame[0:4])
5       2     cmd_id    = 0x0310, little-endian
7       300   data      = 1B inner seq + 299B H264
307     2     CRC16     = CRC16(frame[0:307]), little-endian
```

data 内部布局：

```text
data[0]      inner seq，uint8，0..255 回绕
data[1:300]  299B H264 Annex-B bytes
```

C++ 协议常量位置：

```text
sender/protocol/serial_frame.hpp
```

CRC 模型：

```text
CRC8 normal polynomial      0x31
CRC8 reflected polynomial   0x8C
CRC8 init                   0xFF
CRC8 algorithm              reflected right shift
CRC8 covered bytes          frame[0:4]
CRC8 stored byte            frame[4]

CRC16 normal polynomial     0x1021
CRC16 reflected polynomial  0x8408
CRC16 init                  0xFFFF
CRC16 algorithm             reflected right shift
CRC16 covered bytes         frame[0:307]
CRC16 stored bytes          frame[307:309], little-endian
```

C++ 构帧和 Python local_test parser 必须保持一致：

```text
C++     sender/protocol/serial_frame.cpp
C++     sender/protocol/crc8.hpp
C++     sender/protocol/crc16.hpp
Python  sender/local_test/serial_parser.py
```

协议来源是 `docs/` 下 RoboMaster 2026 通信协议 PDF。

## receiver 包

`receiver` 是 Python ROS2 包，使用 `ament_python`。

重要文件：

```text
receiver/setup.py                         安装和 console scripts
receiver/launch/receiver.launch.py        launch 入口
receiver/config/receiver_params.yaml      receiver 唯一配置文件
receiver/receiver/video_decoder_node.py   ROS 节点、解码和显示
receiver/receiver/mqtt_client.py          paho MQTT 和 protobuf 解析
receiver/receiver/stats_tracker.py        1 秒统计窗口和 seq 统计
receiver/receiver/proto/                  CustomByteBlock protobuf
```

console scripts：

```text
receiver_node = receiver.video_decoder_node:main
decoder_node  = receiver.video_decoder_node:main
```

launch 使用 `receiver_node`。`decoder_node` 只是保留的别名。

### receiver 配置

配置文件：

```text
receiver/config/receiver_params.yaml
```

当前读取：

```text
broker_host
broker_port
topic
client_id
mqtt_queue_size
display
display_scale
crosshair_offset_x
crosshair_offset_y
crosshair_width
```

receiver 不区分 local/match 模式，不在 launch 或节点里交互提问。比赛或本地测试都改 YAML。

receiver 通过 `_required_param()` 声明并读取参数。缺少 config 参数时应该直接报错，不应该在代码里藏业务默认值。

### MQTT 接收

`MqttReceiver` 使用 `paho-mqtt 2.1.0` callback API v2，不兼容旧版 callback API。

连接行为：

```text
启动时尝试连接 broker
连接失败不会退出
timer 中每约 1 秒尝试重连
断开日志约 3 秒节流打印
连接成功后 subscribe(topic, qos=0)
```

消息行为：

```text
payload 解析为 CustomByteBlock
要求 block.data 字段存在
要求 len(block.data) == 300
队列满时丢一个最旧包，再放入最新包
```

protobuf schema 在 sender 和 receiver 各有一份：

```proto
message CustomByteBlock {
  optional bytes data = 1;
}
```

改 proto 时必须两边一起更新生成文件。

### 解码和显示

`VideoDecoderNode`：

```text
创建 av.CodecContext h264 decoder
thread_type = FRAME
尝试设置 LOW_DELAY
每 1ms poll MQTT queue
```

每个 300B data：

```text
seq = data[0]
h264_slice = data[1:]
stats.record(seq, len(h264_slice))
重复 seq：丢弃，不解码
seq gap：统计 missing，reset decoder，然后处理当前 slice
正常：parse/decode h264_slice
```

seq 规则：

```text
diff = (seq - last_seq) & 0xFF
diff == 0  duplicate
diff > 1   gap = diff - 1
```

不做乱序缓存。设计上认为链路应保持顺序；重复包丢弃，跳包 reset decoder。

显示：

```text
decoded frame -> bgr24 ndarray
display=true 时进入 3 帧显示队列
显示窗口名 "Sniper Receiver"
按 display_scale 放大
画准星 overlay
按 q 退出
```

`display=false` 时，每 60 个 decoded frame 打印一次计数。

receiver 不保存 debug dump 图片。

### receiver stats

统计窗口 1 秒，直接硬编码在 `StatsTracker.record()` 的判断处。

格式：

```text
[receiver stats]: rate=49.8pkt/s data=14.89kB/s packets=1000 gaps=0 dups=0 errors=0
```

字段：

```text
rate      当前窗口非重复包率
data      当前窗口 H264 bytes / second / 1000
packets   累计非重复包数
gaps      累计 seq 缺口数
dups      累计重复包数
errors    累计 decode exception 数
```

## README 维护规则

README 面向使用者，只写怎么配置和运行。

README 保留：

```text
依赖安装
sender 配置文件路径
receiver 配置文件路径
start.sh 用法
手动 colcon/ros2 launch 命令
/video_stream 旧解码器验证命令
MQTTX 连接基础信息
```

README 不展开参数解释，不写实现原因，不写架构争论。实现细节放本文件。

## 改动检查

代码改动后按影响范围运行最小检查。

常用检查：

```bash
colcon build --packages-select sender receiver
python3 -m py_compile \
  sender/local_test/main.py \
  sender/local_test/mqtt_server.py \
  sender/local_test/serial_parser.py \
  receiver/receiver/video_decoder_node.py \
  receiver/receiver/mqtt_client.py \
  receiver/receiver/stats_tracker.py
```

文档改动后至少确认：

```text
AGENTS.md 是中文
README 只写使用方式
旧 debug 命名和 debug dump 说明没有恢复
config 参数说明和 YAML、代码实际读取一致
```
