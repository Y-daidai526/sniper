# Sniper 0x0310 图传适配

把低带宽 H264 Annex-B 字节流封装到 RoboMaster 裁判系统自定义图传链路 `0x0310` 中。主链路是：

```text
sender: Hik camera -> x264 -> 299B H264 + 1B seq -> 309B serial frame
debug:  309B serial frame -> local Python MQTT broker -> CustomByteBlock
receiver: MQTT CustomByteBlock -> PyAV H264 decode -> OpenCV display
```

`Pacific_doorlock_sniper/` 只作为参考代码，不需要作为本项目构建目标。

## 环境依赖

机器人端：

```bash
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  gstreamer1.0-tools libopencv-dev
```

还需要海康 MVS SDK，默认代码按 `/opt/MVS` 查找头文件和库。

Python 依赖：

```bash
pip install -r sender/debug/requirements.txt
pip install -r receiver/requirements.txt
```

sender debug 进程会用 `amqtt` 直接启动本地 broker，不需要单独启动 mosquitto。

## 构建

在 ROS2 工作区中只构建新包：

```bash
colcon build --packages-select sender receiver
source install/setup.bash
```

不要把 `Pacific_doorlock_sniper/` 当作必需构建目标。`/video_stream` 兼容调试路径可能需要原 `doorlock_sniper/msg/VideoPacket` 的运行时 type support；缺失时 sender 会禁用该兼容发布，但 0x0310 串口/debug/MQTT 主链路仍应继续工作。

## 运行

正式启动方式使用 `ros2 launch`。

机器人端：

```bash
ros2 launch sender sender.launch.py
```

操作手 PC：

```bash
ros2 launch receiver receiver.launch.py
```

receiver 启动后会提示选择：

- `local`：连接 `receiver/config/receiver_params.yaml` 中的 `local_broker_host`，默认 `127.0.0.1`，使用 `local_client_id`。
- `match`：连接 `match_broker_host`，默认 `192.168.12.1`，继续输入机器人 ID 作为 MQTT client id。

诊断时也可以用 `ros2 run`，但这不是正式启动方式：

```bash
ros2 run sender sender_node --ros-args --params-file install/sender/share/sender/config/sender_params.yaml
ros2 run receiver receiver_node --ros-args --params-file install/receiver/share/receiver/config/receiver_params.yaml
```

## 原解码器兼容验证

sender 会尝试发布 150B `VideoPacket` 兼容流到 `/video_stream`。如果当前 ROS2 环境中有原 `doorlock_sniper/msg/VideoPacket` type support 和原 decoder，可以这样单独验证编码管线：

```bash
ros2 launch sender sender.launch.py
ros2 run doorlock_decoder video_decoder_node --ros-args -p topic:=/video_stream
```

如果 sender 打印 `VideoPacket compatibility publisher disabled`，说明缺少兼容消息的运行时 type support；这不影响 0x0310 主链路。

## 关键参数

所有运行参数在 YAML 中，不在代码里改地址或调参。

sender runtime：`sender/config/sender_params.yaml`

| 参数 | 说明 |
|---|---|
| `debug_mqtt_host` | 本地 debug broker 地址，默认 `127.0.0.1` |
| `debug_mqtt_port` | 本地 debug broker 端口，默认 `3333` |
| `debug_mqtt_topic` | MQTT topic，默认 `CustomByteBlock` |
| `enable_serial` | 是否写 USB CDC 串口 |
| `enable_debug_bridge` | 是否启动 Python debug bridge |

sender encoder：

| 参数 | 说明 |
|---|---|
| `target_bitrate` | x264 目标码率，单位 kbps |
| `output_size` | 编码输出宽高，默认 300x300 |
| `output_fps` | 编码输入帧率上限，最大 60 |
| `enable_video_stream` | 是否尝试发布 `/video_stream` 兼容流 |
| `serial_max_rate_hz` | 0x0310 主链路发送频率上限，默认 50 |
| `max_tx_delay_s` | serial backlog 最大保留时延 |

receiver：`receiver/config/receiver_params.yaml`

| 参数 | 说明 |
|---|---|
| `local_broker_host` | 本地测试 MQTT host，默认 `127.0.0.1` |
| `match_broker_host` | 比赛 MQTT host，默认 `192.168.12.1` |
| `broker_port` | MQTT 端口，默认 `3333` |
| `topic` | MQTT topic，默认 `CustomByteBlock` |
| `local_client_id` | 本地测试 client id |
| `mqtt_queue_size` | receiver 内部 MQTT 队列长度 |

## 协议与速率

0x0310 串口帧固定 309B：

```text
SOF(1) + data_len(2) + seq(1) + CRC8(1) + cmd_id(2) + data(300) + CRC16(2)
```

`data` 固定 300B：

```text
data[0]      内部 seq，uint8，0..255 回绕
data[1:300]  299B H264 Annex-B 切片
```

主链路瓶颈是 300B data、50Hz；H264 有效载荷上限约 `299B * 50Hz = 14.95kB/s`。串口/debug 路径按 `serial_max_rate_hz` 限速，默认 50Hz；`/video_stream` 只是本地兼容调试路径，不按 50Hz 设计。

CRC 实现见 `sender/protocol/crc8.hpp`、`sender/protocol/crc16.hpp` 和 `sender/debug/serial_parser.py`：

- CRC8：正常多项式 `0x31`，反射实现 `0x8C`，初始值 `0xFF`。
- CRC16：正常多项式 `0x1021`，反射实现 `0x8408`，初始值 `0xFFFF`，小端写入。
