# RoboMaster 自定义客户端图传适配

本仓库只需要构建和运行 `sender/`、`receiver/`。`Pacific_doorlock_sniper/` 只作为参考源码。

## 1. 安装依赖

```bash
sudo apt install -y \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  gstreamer1.0-tools libopencv-dev \
  python3-pip python3-av python3-opencv
```

安装海康 MVS SDK，并确认 `/opt/MVS/include` 和 `/opt/MVS/lib/64` 存在。

MQTT 客户端统一使用 `paho-mqtt`。`amqtt` 用于 sender local_mqtt 内置 MQTT broker：

```bash
pip install paho-mqtt amqtt
```

Python 3.12 若因 PEP 668 禁止直接使用 `pip` 安装系统包，请使用：

```bash
pip install --break-system-packages paho-mqtt amqtt
```


## 2. 配置 sender

配置文件：`sender/config/sender_params.yaml`。

## 3. 配置 receiver

配置文件：`receiver/config/receiver_params.yaml`。

当前 receiver 配置连接 `192.168.12.1:3333`。连接 sender 内置 broker 做本机测试时，把 `broker_host` 改为 `127.0.0.1`。

## 4. 一键构建和运行

编译并运行一端：

```bash
./sender/start.sh
./receiver/start.sh
```

脚本切换到各自包目录，并在该目录下独立构建和运行。
