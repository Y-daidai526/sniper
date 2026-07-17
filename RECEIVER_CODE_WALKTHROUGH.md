# Receiver 启动与代码调用流程

本文按实际执行顺序梳理 `receiver`：从 `start.sh` 启动，到 MQTT 收包、H264 解码、统计和 OpenCV 显示，并列出后续优化时值得重点测量的位置。

## 总体调用链

```text
receiver/start.sh
  -> colcon build --packages-select receiver
  -> source receiver/install/setup.bash
  -> ros2 launch receiver receiver.launch.py
  -> launch 查找 receiver_params.yaml
  -> 启动 setup.py 注册的 receiver_node
  -> receiver.video_decoder_node:main
  -> VideoDecoderNode.__init__
       -> 读取必需 ROS 参数
       -> 创建 MqttReceiver
       -> 启动 Paho MQTT 网络线程
       -> 创建 PyAV H264 decoder
       -> 可选启动 OpenCV 显示线程
       -> 创建 1ms ROS timer
  -> rclpy.spin(node)
  -> timer 周期调用 _poll_mqtt()
       -> 从 MQTT 队列取 300B data
       -> seq 统计和重复/丢包判断
       -> 将 299B H264 喂给 PyAV
       -> 解码帧进入显示队列
  -> 显示线程缩放、画准星并显示
```

## 1. `receiver/start.sh`

入口文件：`receiver/start.sh`。

脚本首先检查参数。当前不接受命令行参数，所有业务配置都从 YAML 读取。

脚本随后将工作目录切换到 `receiver/`：

```bash
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${PACKAGE_DIR}"
```

因此独立构建产物位于：

```text
receiver/build/
receiver/install/
receiver/log/
```

构建及启动命令为：

```bash
colcon build --packages-select receiver
source receiver/install/setup.bash
ros2 launch receiver receiver.launch.py
```

每次运行 `start.sh` 都会先构建，然后持续运行 receiver 节点。

## 2. `setup.py` 和可执行入口

`receiver/setup.py` 将以下 Python package 安装到工作空间：

```text
receiver
receiver.proto
```

同时安装 ROS2 需要的资源：

```text
package.xml
launch/receiver.launch.py
config/receiver_params.yaml
```

console script 定义为：

```python
receiver_node = receiver.video_decoder_node:main
decoder_node = receiver.video_decoder_node:main
```

因此 launch 中启动 `receiver_node`，实际调用的是：

```python
receiver.video_decoder_node.main()
```

`decoder_node` 只是保留的兼容别名。

## 3. launch 和配置文件选择

入口文件：`receiver/launch/receiver.launch.py`。

配置文件按以下顺序查找：

```text
<当前目录>/config/receiver_params.yaml
<当前目录>/receiver/config/receiver_params.yaml
<install share>/receiver/config/receiver_params.yaml
```

使用 `receiver/start.sh` 时，当前目录已经是 `receiver/`，因此优先读取源码中的：

```text
receiver/config/receiver_params.yaml
```

从仓库根目录手动 launch 时，第二个候选路径命中同一份源码配置。只有源码配置不存在时才回退到 install share。

launch 还会：

- 设置无时间戳的 ROS 日志格式；
- 打印实际使用的配置文件路径；
- 启动名为 `video_decoder` 的 ROS 节点；
- 将整份 YAML 作为节点参数传入。

YAML 顶层名称必须与节点名称一致：

```yaml
video_decoder:
  ros__parameters:
```

launch 不保存业务参数默认值。

## 4. Receiver 配置

配置文件：`receiver/config/receiver_params.yaml`。

MQTT 参数：

```text
broker_host
broker_port
topic
client_id
mqtt_queue_size
```

显示参数：

```text
display
display_scale
crosshair_offset_x
crosshair_offset_y
crosshair_width
```

`video_decoder_node.py` 使用 `_required_param()` 声明并读取参数。代码中不提供另一份业务默认值，缺少参数会直接报错。

字符串参数还会检查是否为空。整数参数目前缺少完整的业务范围验证，后续可考虑集中检查端口、队列容量、缩放倍率等参数。

## 5. ROS 节点生命周期

`receiver/receiver/video_decoder_node.py` 中的 `main()` 执行：

```python
rclpy.init(args=args)
node = VideoDecoderNode()
rclpy.spin(node)
```

ROS2 在 receiver 中主要负责：

- 参数加载；
- 日志；
- 节点生命周期；
- timer 调度；
- launch 集成。

视频数据不经过 ROS topic，而是直接通过 MQTT 接收。

退出时依次销毁 timer、断开 MQTT、停止显示线程、销毁 ROS 节点并关闭 rclpy。

## 6. `VideoDecoderNode` 初始化

构造函数主要完成以下工作：

1. 读取全部必需参数。
2. 创建 `MqttReceiver` 并尝试连接 broker。
3. 创建 PyAV H264 decoder。
4. 创建 `StatsTracker`。
5. `display=true` 时启动 OpenCV 显示线程。
6. 创建周期为 1ms 的 ROS timer。

初次 MQTT 连接失败不会退出节点，timer 会继续触发重连。

`MqttReceiver.connect()` 返回成功只表示连接请求没有同步抛出异常。broker 是否真正接受连接，要等 Paho 网络线程调用 `_on_connect()` 才能确定。

## 7. 三个并发执行域

Receiver 当前有三个主要执行域：

| 执行域 | 主要工作 | 数据交接 |
|---|---|---|
| Paho 网络线程 | MQTT socket、protobuf 解析、300B 校验 | MQTT Queue |
| ROS executor 主线程 | 重连、seq 统计、PyAV parse/decode | 显示 Queue |
| OpenCV 显示线程 | resize、准星 overlay、窗口显示 | GUI |

代码没有为视频流显式加 mutex，而是使用线程安全的 `queue.Queue` 交接数据。

## 8. MQTT 连接和收包

入口文件：`receiver/receiver/mqtt_client.py`。

首次连接执行：

```python
client.connect(host, port, keepalive=30)
client.loop_start()
```

`loop_start()` 创建 Paho 后台网络线程。以下回调都运行在该线程中：

```text
_on_connect()
_on_disconnect()
_on_message()
```

连接成功后订阅：

```python
client.subscribe(topic, qos=0)
```

QoS 0 不保证 MQTT 层重传，延迟较低，丢包由 data 内部的 seq 检测。

消息使用 protobuf `CustomByteBlock`：

```proto
message CustomByteBlock {
    optional bytes data = 1;
}
```

接收端要求：

```text
block.data 字段存在
len(block.data) == 300
```

300B data 布局为：

```text
data[0]      inner seq
data[1:300]  299B H264 Annex-B bytes
```

Receiver 不处理完整的 309B 裁判系统帧。完整帧已经由 sender 的 local MQTT 链路校验并剥除协议头尾。

## 9. MQTT 队列策略

Paho 网络线程将合法的 300B data 放进 `queue.Queue`。

队列未满时直接放入。队列满时：

```text
删除一个最旧包
放入当前最新包
```

该策略限制了内存上限，但只在队列完全满时才开始追赶实时数据。

当前默认容量为 1000。若发送速率为 50 包/秒，理论上可积累约 20 秒数据。因此队列容量不仅影响内存，还直接影响最坏播放延迟。

## 10. MQTT 重连

ROS timer 每次处理数据前调用 `_retry_mqtt_if_needed()`。

重连逻辑约每 1 秒允许发起一次连接尝试，连接失败日志约每 3 秒打印一次。连接失败不会停止解码节点。

Paho 自己也设置了：

```python
reconnect_delay_set(min_delay=1, max_delay=5)
```

后续优化时应确认手动重连逻辑与 Paho 内部重连行为是否存在重复尝试或状态竞争。

## 11. ROS timer 和数据处理循环

timer 周期为 1ms，回调函数是 `_poll_mqtt()`。

回调内部使用循环把当前队列一直取到为空：

```python
while True:
    data = self._mqtt.get_data(0.0)
    if data is None:
        break
```

这不是严格的 1000Hz 实时任务。如果一次回调处理时间超过 1ms，下一次回调只能等待当前回调结束。

如果 MQTT 持续入队且解码速度较慢，`while True` 可能长时间不返回，导致 ROS executor 中其他 callback 无法及时运行。

## 12. seq 判断和统计

每个 data 被拆成：

```python
seq = data[0]
h264_slice = data[1:]
```

seq 使用 uint8 环绕算法：

```python
diff = (seq - last_seq) & 0xFF
```

判断规则：

```text
diff == 0   重复包，统计 dups，不送入 decoder
diff == 1   正常连续包
diff > 1    缺少 diff - 1 个包，统计 gaps 并 reset decoder
```

例如：

```text
255 -> 0    diff=1，正常回绕
10 -> 10    diff=0，重复
10 -> 13    diff=3，缺2包
```

当前不做乱序缓存。迟到包、严重乱序或 sender 重启可能被识别成很大的 gap。

`StatsTracker` 约每秒打印：

```text
rate
data
packets
gaps
dups
errors
```

统计输出由 `record()` 中的数据到达触发。如果完全收不到数据，不会独立打印零速率窗口。

## 13. H264 parser 和 decoder

解码器通过以下方式创建：

```python
av.CodecContext.create("h264", "r")
codec.thread_type = "FRAME"
codec.flags |= LOW_DELAY
```

299B MQTT chunk 不等于一个视频帧，也不一定是一个完整 NAL 单元。真实处理层次是：

```text
299B MQTT H264 chunk
  -> codec.parse()
完整或可提交的压缩 packet
  -> codec.decode()
解码后的 VideoFrame
```

parser 会在解码上下文中跨 MQTT 包缓存 H264 字节。

发现 seq gap 时，节点销毁旧 decoder 并创建新 decoder，然后继续处理当前 299B chunk。当前 chunk 可能从 NAL 单元中间开始，因此通常要等后续 SPS、PPS 和 IDR 才能恢复图像。

sender 的 repeat headers 和关键帧周期会直接影响丢包后的恢复速度。

解码异常会增加 `errors`，详细异常只用 ROS debug 级别输出。FFmpeg 全局日志被设置为 `PANIC`，用于避免丢包时刷出大量重复错误。

## 14. 解码帧处理

每个解码帧先执行：

```python
img = frame.to_ndarray(format="bgr24")
```

这会进行像素格式转换和 ndarray 内存分配。

`display=true` 时，BGR 图像进入容量为 3 的显示队列。

`display=false` 时，当前代码仍然执行 BGR 转换，只是不显示，并且每 60 个 decoded frame 打印一次计数。这是一个直接的 CPU 和内存优化点：无显示模式可以只统计 frame，不创建 ndarray。

## 15. OpenCV 显示线程

显示线程执行：

```text
从显示队列取 BGR frame
-> 按 display_scale 使用 INTER_NEAREST 放大
-> 绘制横纵准星和中心圆
-> cv2.imshow()
-> cv2.waitKey(1)
```

准星偏移单位是放大后的显示像素。

当前显示队列满时直接丢弃刚解码出的最新帧，保留队列中的旧帧。对于实时操控画面，这会增加视觉延迟。更合适的策略通常是：

```text
队列满 -> 删除最旧帧 -> 放入最新帧
```

按 `q` 时，显示线程调用 `rclpy.shutdown()`，主线程中的 `rclpy.spin()` 随后退出。

## 16. 清理过程

`destroy_node()` 当前执行：

```text
销毁 ROS timer
断开 MQTT 并停止 Paho loop
尝试向显示队列放入 None
等待显示线程最多 1 秒
销毁 ROS node
```

如果显示队列已满，`None` 可能无法放入。一般情况下显示线程仍会因 `rclpy.ok()` 变为 false 而退出，但关闭信号可以做得更确定。

## 17. 建议优先分析的优化点

### 17.1 MQTT 队列延迟

`mqtt_queue_size=1000` 可能隐藏很长的 backlog。建议先增加以下指标：

```text
当前 queue depth
最大 queue depth
queue 满导致的丢包数
数据在队列中的驻留时间
```

再决定缩小队列，还是增加主动追赶最新数据的策略。

### 17.2 限制单次 timer 工作量

当前 `_poll_mqtt()` 会一直处理到队列为空。可以评估改为：

```text
每次最多处理 N 个包
或每次最多占用若干毫秒
```

这样可以避免长期占用 ROS executor。

### 17.3 显示队列保留最新帧

当前满时丢最新帧，建议评估改成丢最旧帧，以降低操作画面的延迟。

### 17.4 无显示模式跳过 BGR 转换

`display=false` 时可以不调用 `frame.to_ndarray()`。这属于低风险、收益明确的优化。

### 17.5 测量丢包恢复时间

建议记录：

```text
decoder reset 次数
发生 gap 的时间
gap 后第一张成功解码图像的时间
恢复耗时
```

然后再决定调整 receiver reset 策略，还是调整 sender 的 IDR、SPS/PPS 周期。

### 17.6 减少不必要的数据转换

当前主要转换和复制包括：

```text
protobuf data -> Python bytes
H264 bytes -> PyAV parser
VideoFrame -> BGR ndarray
BGR ndarray -> resize 后 ndarray
```

300B 数据复制通常不是首要瓶颈。应优先测量 H264 解码、BGR 转换和图像 resize。

### 17.7 改善可观测性

建议增加但不刷屏的聚合指标：

```text
MQTT queue depth/drops
display queue drops
decoder resets
decoded FPS
display FPS
收包到显示的估算延迟
gap 后恢复耗时
```

## 18. 推荐优化顺序

建议先测量，再改变结构：

1. 增加 MQTT 队列深度、队列丢包和 decoded FPS 指标。
2. 将 `mqtt_queue_size` 临时降到约 50～100，观察卡顿和延迟。
3. 将显示队列改为保留最新帧，观察操作跟手程度。
4. 在 `display=false` 时跳过 BGR ndarray 转换。
5. 测量 gap 后恢复时间，再联合调整 sender 编码策略。
6. 最后考虑 timer 预算、decoder 生命周期和更大范围的线程结构调整。
