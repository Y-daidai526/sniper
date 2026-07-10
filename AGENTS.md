# AGENTS.md - Sniper 0x0310 图传适配开发规范

## 1. 当前目标

把 `Pacific_doorlock_sniper/` 中已经验证过的低带宽 H264 编码思路，迁移到新的 `sender/` 和 `receiver/` 中，用 RoboMaster 裁判系统图传链路命令 `0x0310` 替换原 ROS2 Topic 主传输层。

目标链路：

```text
sender/ ROS2 C++
  Hik camera -> GStreamer x264 -> H264 Annex-B bytes
    -> serial/debug path: 299B H264 + 1B seq -> 300B data -> 309B referee serial frame
    -> debug bridge: 309B frame -> fork/exec Python stdin -> local MQTT CustomByteBlock
    -> optional compatibility path: 150B VideoPacket-compatible /video_stream

receiver/ ROS2 Python
  configured MQTT CustomByteBlock -> 300B data -> seq/loss stats -> PyAV decode -> display
```

`Pacific_doorlock_sniper/` 只作为参考来源。不要修改它，不要把它作为构建目标，不要让 sender/receiver 的主链路依赖它。需要复用时复制已验证实现到 `sender/` 或 `receiver/`，再只做必要协议适配。

## 2. 硬约束

- 顶层只维护新的 `sender/` 和 `receiver/`；不要新增 `compat/`、`common/`、`doorlock_sniper/` 等顶层目录。
- 不修改 `Pacific_doorlock_sniper/` 下任何文件。
- 不编译 `Pacific_doorlock_sniper/`；README 和构建命令必须避免把原项目作为必要构建目标。
- sender 和 receiver 的正式启动方式都是 `ros2 launch`。README 可以同时提供诊断用 `ros2 run` 方法。
- 所有可设置项必须放在各自 config YAML 中，尤其 MQTT host/port/topic/client id 相关配置；不要把运行配置硬编码在逻辑里。
- 不保留“写进 config 但代码没读”的参数。
- 尽量不要给内部函数使用默认参数，避免调用方对真实行为产生歧义。

## 3. 依赖策略

- 可以复制参考：相机采集、图像预处理、GStreamer x264 参数、PyAV/OpenCV 解码显示结构。
- 不复制构建关系：新项目不依赖原 `doorlock_sniper`、`hik_camera`、`doorlock_decoder` 包来完成主链路。
- `/video_stream` 兼容发布是可选调试路径。实现使用 ROS2 `GenericPublisher` 和手写 CDR 数据，类型名为 `doorlock_sniper/msg/VideoPacket`。
- `GenericPublisher` 仍需要运行环境 source 到 `doorlock_sniper/msg/VideoPacket` type support；可以来自原项目或另一个独立构建的同名同定义消息包。缺失时只禁用兼容发布，不得影响 0x0310 串口/debug/MQTT 主链路。
- 不再使用临时 ROS2 MQTT bridge：不要使用 `std_msgs/UInt8MultiArray`、`serial_data` topic、`mqtt_bridge_node.py` 作为主链路。

## 4. sender/

sender 是机器人端 C++ ROS2 程序，正式入口：

```bash
ros2 launch sender sender.launch.py
```

要求：

- 一个 sender 进程内完成相机、编码、分包、串口、debug bridge 连接，避免多进程回调错位。
- 相机基于海康 MVS SDK，发布图像给编码器；相机采集行为尽量照抄 Pacific 已验证实现。YAML 只保留现场确实需要调的 `use_sensor_data_qos`、`exposure_time`、`gain`、`camera_name`、`camera_info_url`。
- 编码器输出连续 H264 Annex-B 字节流。
- appsink 拉出的 H264 字节进入两个独立 buffer：
  - `serial_stream_buffer_`：每次取 299B，加 1B 内部 seq，形成 300B data。
  - `ros2_stream_buffer_`：每次取 150B，发布兼容 `VideoPacket` 的 `/video_stream`，只用于本地调试和原解码器验证。
- serial/debug path 必须构造完整 309B 串口帧，不能只传 300B data。
- USB CDC 串口支持热插拔，持续扫描 `/dev/ttyACM*` 和 `/dev/ttyUSB*`；无串口时编码器继续运行。
- debug bridge 使用 `fork()` + `exec()` 启动 `sender/debug/main.py`，父进程通过 stdin pipe 写入 309B frame。
- debug Python 进程使用 `amqtt` 直接启动本地 MQTT broker，地址来自 `sender/config/sender_params.yaml`，默认配置为 `127.0.0.1:3333`。
- debug Python 进程验证 CRC 后，把 300B data 封装为 protobuf `CustomByteBlock` 并发布到本地 broker。

## 5. receiver/

receiver 是操作手 PC 端 Python ROS2 程序，正式入口：

```bash
ros2 launch receiver receiver.launch.py
```

要求：

- 不再区分 local-test/match 运行模式，不在 launch 或节点里提示用户选择。
- MQTT broker host/port/topic/client id 都来自 `receiver/config/receiver_params.yaml`；默认 host 为 `127.0.0.1`，比赛时直接修改 YAML。
- MQTT 订阅 `CustomByteBlock`，反序列化 protobuf 后必须得到 300B data。
- `data[0]` 是内部 `uint8` seq，用于包速率、数据速率、丢包率、重包统计。
- `data[1:300]` 是 299B H264 Annex-B 切片，直接喂给 PyAV parser/decoder。
- seq 重复时丢弃重包；seq 不连续时统计丢包并 reset decoder；不做乱序重排缓存。

## 6. 0x0310 协议

串口帧固定 309B：

```text
SOF      1B   0xA5
data_len 2B   300 / 0x012C, little-endian
seq      1B   referee serial frame seq
CRC8     1B   covers first 4B: SOF + data_len + seq
cmd_id   2B   0x0310, little-endian
data     300B 1B inner seq + 299B H264
CRC16    2B   covers SOF through data end, little-endian
```

`0x0310` data 内部布局：

```text
data[0]      uint8 inner seq, wraps 0..255
data[1:300]  299B H264 Annex-B bytes
```

CRC 要求来自 `docs/UTF-8__RoboMaster 2026 机甲大师高校系列赛通信协议 V2.0.0（20260626）.pdf` 附录一；该 PDF 带加密标记但空密码可读。

- CRC8：正常多项式 `0x31`，即 `G(x)=x8+x5+x4+1`；协议实现使用反射右移模型，等价反射多项式 `0x8C`，初始值 `0xFF`。
- CRC8 覆盖 frame header 前 4B，并写入 header 第 5B。
- CRC16：正常多项式 `0x1021`，即 `G(x)=x16+x12+x5+1`；协议实现使用反射右移模型，等价反射多项式 `0x8408`，初始值 `0xFFFF`。
- CRC16 覆盖整包除最后 2B CRC 字段，低字节在前写入 frame tail。
- C++ 构帧和 Python debug parser 必须使用同一套 CRC 模型，输出一致。

## 7. 速率与限流

- 0x0310 链路瓶颈是 300B data 段、50Hz 频率上限。
- 主链路理论 data 上限：`300B * 50Hz = 15.0kB/s`。
- 真正 H264 有效载荷理论上限：`299B * 50Hz = 14.95kB/s`。
- 原项目约 14kB/s 峰值不是 `150B * 50Hz`，而是原滑动窗口限速叠加 150B 包大小后得到的结果，包率约 90Hz 量级。
- 新实现中 serial/debug path 使用 `serial_max_rate_hz`，默认 50Hz；`/video_stream` 是本地兼容调试链路，不受 0x0310 50Hz 限制，也不得影响 serial path。
- serial buffer 不足 299B 时等待，不补零。
- 队列超过 `max_tx_delay_s` 对应 backlog 时，允许尽量按 Annex-B start code 对齐后丢弃旧数据，避免延迟无限增长。

## 8. Config 契约

`sender/config/sender_params.yaml` 只包含 sender 实际读取的参数：

- `sender_runtime`：`enable_serial`、`enable_debug_bridge`、`debug_start_broker`、`debug_mqtt_host`、`debug_mqtt_port`、`debug_mqtt_topic`、`debug_script_dir`。
- `hik_camera`：`use_sensor_data_qos`、`exposure_time`、`gain`、`camera_name`、`camera_info_url`。
- `video_encoder`：`input_topic`、`video_stream_topic`、`enable_video_stream`、编码参数、预处理参数、`bandwidth_limit_kbytes`、`bandwidth_window_s`、`serial_max_rate_hz`、`max_tx_delay_s`、display/debug dump 参数。

`receiver/config/receiver_params.yaml` 只包含 receiver 实际读取的参数：

- MQTT：`broker_host`、`broker_port`、`topic`、`client_id`、`mqtt_queue_size`。
- display：`display`、`display_scale`、`crosshair_offset_x`、`crosshair_offset_y`、`crosshair_width`。
- stats/debug：`stats_interval_s`、`debug_dump_enable`、`debug_dump_every_n_frames`、`debug_dump_dir`。

协议固定项不要放进 config：`cmd_id=0x0310`、`frame_size=309`、`data_len=300`、`h264_slice_size=299`、`VideoPacket payload=150`。

## 9. README 要求

README 最后更新，必须包含：

- 环境依赖。
- 构建方式，明确不需要编译 Pacific 原项目。
- 推荐运行方式：sender/receiver 都用 `ros2 launch`。
- 诊断方式：提供 `ros2 run` 命令，但标注不是正式启动方式。
- 原解码器兼容验证命令，并注明该路径依赖原 `VideoPacket` type support；缺失时主链路仍应工作。
- 参数说明必须和 YAML 实际参数一致。
