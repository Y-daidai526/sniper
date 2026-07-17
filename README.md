# RoboMaster 自定义客户端图传适配

本仓库包含机器人端 `sender/` 和操作手端 `receiver/` 两个 ROS 2 包。

## 1. 安装依赖

```bash
sudo apt install -y \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  gstreamer1.0-tools libopencv-dev \
  python3-pip python3-av python3-opencv
```

安装海康 MVS SDK，并确认 `/opt/MVS/include` 和 `/opt/MVS/lib/64` 存在。

MQTT 客户端固定使用 `paho-mqtt 2.1.0`，sender 内置 broker 使用 `amqtt`：

```bash
pip install paho-mqtt==2.1.0 amqtt
```

Python 3.12 若因 PEP 668 禁止直接使用 `pip` 安装系统包，请使用：

```bash
pip install --break-system-packages paho-mqtt==2.1.0 amqtt
```

## 2. 配置 sender

配置文件：`sender/config/sender_params.yaml`。

## 3. 配置 receiver

配置文件：`receiver/config/receiver_params.yaml`。

连接 sender 内置 broker 做本机测试时，将 receiver 的 broker 地址设为 `127.0.0.1`。

运行后的主要图像话题：

```text
/sender/image_raw              海康相机原始图像
/receiver/video_stream/raw     receiver 原始解码图像
/receiver/video_stream         缩放并绘制准星后的图像
```

## 4. 一键构建和运行

编译并运行一端：

```bash
./start_sender.sh
./start_receiver.sh client_id:=1
```

`start_receiver.sh` 未传入 `client_id` 时会在启动阶段提示输入；同时运行多个 receiver 时应使用不同的非空 ID。两个脚本分别只构建当前端，构建产物统一位于仓库根目录的 `build/`、`install/` 和 `log/`。

## 5. 手动构建和运行

```bash
colcon build --symlink-install --packages-select sender receiver
source install/setup.bash

ros2 launch sender sender.launch.py
ros2 launch receiver receiver.launch.py client_id:=1
```

只运行一端时，可以将 `--packages-select` 后的包名改为 `sender` 或 `receiver`。

## 6. MQTTX 本地检查

sender 默认内置 broker 连接信息：

```text
host: 127.0.0.1
port: 3333
topic: CustomByteBlock
qos: 0
```
