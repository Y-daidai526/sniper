# Sniper 0x0310 图传适配

链路：

```text
sender: Hik camera -> x264 -> 299B H264 + 1B seq -> 309B 0x0310 frame
debug:  309B frame -> Python stdin -> MQTT CustomByteBlock
receiver: MQTT CustomByteBlock -> PyAV decode -> OpenCV display
```

`Pacific_doorlock_sniper/` 只作为参考代码，不作为本项目构建目标。

## 1. 安装依赖

```bash
sudo apt install -y \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  gstreamer1.0-tools libopencv-dev \
  python3-pip python3-paho-mqtt python3-av python3-opencv
```

安装海康 MVS SDK，并确认头文件和库在 `/opt/MVS`。

`amqtt` 用于 sender debug 内置 MQTT broker，单独用 pip 装：

```bash
python3 -m pip install --break-system-packages amqtt
```

如果 pip 被 Debian 自带的 `PyYAML` 卡住，改用：

```bash
python3 -m pip install --break-system-packages --ignore-installed PyYAML amqtt
```

## 2. 构建本项目

```bash
colcon build --packages-select sender receiver
source install/setup.bash
```

## 3. 可选：准备 `/video_stream` 兼容类型

如果要用旧解码器验证 `/video_stream`，sender 运行环境需要 `doorlock_sniper/msg/VideoPacket` type support。不要复制消息文件，直接用原项目那份 `doorlock_sniper` 源码编译到根目录默认 `build/ install/ log/`：

```bash
colcon build \
  --base-paths Pacific_doorlock_sniper/src/doorlock_sniper \
  --packages-select doorlock_sniper \
  --allow-overriding doorlock_sniper
source install/setup.bash
```

旧解码器 shell：

```bash
cd Pacific_doorlock_sniper
source install/setup.bash
ros2 run doorlock_decoder decoder_node --ros-args -p topic:=/video_stream
```

推荐先启动旧解码器，再启动 sender。中途订阅 H264 流时，旧解码器可能先打印 `non-existing PPS`，等下一组 SPS/PPS/IDR 后会恢复。

两边可以 source 不同 install；只要类型都叫 `doorlock_sniper/msg/VideoPacket` 且字段一致，就能通信。

## 4. 正式运行

机器人端：

```bash
source install/setup.bash
ros2 launch sender sender.launch.py
```

操作手 PC：

```bash
source install/setup.bash
ros2 launch receiver receiver.launch.py
```

receiver 直接读取 `receiver/config/receiver_params.yaml`：

```yaml
broker_host: "127.0.0.1"
broker_port: 3333
topic: "CustomByteBlock"
client_id: "sniper_receiver_local"
```

比赛时直接改这个 YAML 的 MQTT 地址和 client id。

## 5. 诊断命令

`ros2 run` 只用于诊断，不是正式启动方式：

```bash
ros2 run sender sender_node --ros-args --params-file install/sender/share/sender/config/sender_params.yaml
ros2 run receiver receiver_node --ros-args --params-file install/receiver/share/receiver/config/receiver_params.yaml
```

推荐调通顺序：

1. `sender + 旧解码器`：确认 `/video_stream` 能显示。
2. `sender debug -> MQTT`：确认 debug bridge 能发布 `CustomByteBlock`。
3. `receiver -> MQTT`：确认 receiver 能解码显示。

如果 sender 打印 `VideoPacket compatibility publisher disabled`，说明当前 shell 没有 source 到 `doorlock_sniper/msg/VideoPacket` type support；这不影响 0x0310 串口/debug/MQTT 主链路。

## 6. 配置文件

- `sender/config/sender_params.yaml`
  - `sender_runtime`：串口、debug bridge、debug MQTT 参数。
  - `hik_camera`：曝光、增益、camera info。
  - `video_encoder`：编码参数、预处理参数、`/video_stream` 兼容流、0x0310 发送频率。
- `receiver/config/receiver_params.yaml`
  - MQTT 地址、client id、显示、准心、统计和 dump 参数。

协议固定项不进 YAML：`cmd_id=0x0310`、`frame_size=309`、`data_len=300`、`h264_slice_size=299`、`VideoPacket payload=150`。
