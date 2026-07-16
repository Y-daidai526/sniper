# RoboMaster 自定义客户端图传适配

本仓库只需要构建和运行 `sender/`、`receiver/`。`Pacific_doorlock_sniper/` 只作为参考和可选旧解码器验证来源。

## 1. 安装依赖

```bash
sudo apt install -y \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  gstreamer1.0-tools libopencv-dev \
  python3-pip python3-av python3-opencv
```

安装海康 MVS SDK，并确认 `/opt/MVS/include` 和 `/opt/MVS/lib/64` 存在。

MQTT 客户端统一使用 `paho-mqtt 2.1.0`。`amqtt` 用于 sender local_test 内置 MQTT broker：

```bash
python3 -m pip install --break-system-packages paho-mqtt==2.1.0 amqtt
```

如果 pip 被系统自带 `PyYAML` 卡住：

```bash
python3 -m pip install --break-system-packages --ignore-installed PyYAML amqtt
python3 -m pip install --break-system-packages paho-mqtt==2.1.0
```

## 2. 配置 sender

配置文件：`sender/config/sender_params.yaml`。

## 3. 配置 receiver

配置文件：`receiver/config/receiver_params.yaml`。

默认 MQTT 地址是 `127.0.0.1:3333`。比赛时直接把 YAML 里的 `broker_host`、`broker_port`、`client_id` 改成现场值。

## 4. 一键构建和运行

编译并运行一端：

```bash
./start.sh sender
./start.sh receiver
```

## 5. 手动构建和运行

```bash
colcon build --packages-select sender receiver
source install/setup.bash
ros2 launch sender sender.launch.py
```

另一个终端运行 receiver：

```bash
source install/setup.bash
ros2 launch receiver receiver.launch.py
```

诊断用 `ros2 run`：

```bash
ros2 run sender sender_node --ros-args --params-file install/sender/share/sender/config/sender_params.yaml
ros2 run receiver receiver_node --ros-args --params-file install/receiver/share/receiver/config/receiver_params.yaml
```

## 6. `/video_stream` 旧解码器验证

如果要用原项目旧解码器看 `/video_stream`，先在根目录编译原 `VideoPacket` type support：

```bash
colcon build \
  --base-paths Pacific_doorlock_sniper/src/doorlock_sniper \
  --packages-select doorlock_sniper \
  --allow-overriding doorlock_sniper
source install/setup.bash
```

旧解码器终端：

```bash
cd Pacific_doorlock_sniper
source install/setup.bash
ros2 run doorlock_decoder decoder_node --ros-args -p topic:=/video_stream
```

缺少 `doorlock_sniper/msg/VideoPacket` type support 时，sender 会禁用 `/video_stream` 兼容发布；串口、local_test MQTT、receiver 主链路不受影响。

## 7. MQTTX 查看

连接 sender 内置 broker：

- Host：`127.0.0.1`
- Port：`3333`
- Protocol：`mqtt://`
- MQTT Version：`3.1.1`
- TLS/WebSocket：关闭
- Topic：`CustomByteBlock`

消息体是 protobuf 二进制，MQTTX 只能用于确认 topic 上有数据。
