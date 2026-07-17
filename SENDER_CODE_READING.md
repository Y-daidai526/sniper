# Sender 代码阅读笔记

本文记录 `sender` 从 `start.sh` 到进程启动、节点创建和后台线程运行的真实调用过程，供后续分析和优化使用。

## 1. 总体启动链路

```text
./sender/start.sh
  -> colcon build --packages-select sender
  -> source sender/install/setup.bash
  -> ros2 launch sender sender.launch.py
  -> 启动 sender_node 可执行文件
  -> main()
      |- sender_runtime ROS 节点
      |- hik_camera ROS 节点
      |- video_encoder ROS 节点
      `- SerialSendWorker 普通 C++ 对象
  -> MultiThreadedExecutor::spin()
  -> 持续采集、编码、分包和发送
```

`sender_runtime`、`hik_camera` 和 `video_encoder` 都位于同一个 `sender_node` 进程内，并不是三个独立 ROS 进程。

## 2. start.sh

入口文件：

```text
sender/start.sh
```

执行过程：

1. 使用 `set -eo pipefail`，命令失败时立即退出。
2. 拒绝所有命令行参数。
3. 计算并进入 `sender/` 包目录。
4. 只构建 `sender` 包：

   ```bash
   colcon build --packages-select sender
   ```

5. 加载本次构建生成的 ROS 环境：

   ```bash
   source sender/install/setup.bash
   ```

6. 启动 launch 文件：

   ```bash
   ros2 launch sender sender.launch.py
   ```

因为脚本先进入 `sender/`，所以构建产物分别位于：

```text
sender/build/
sender/install/
sender/log/
```

优化关注点：当前脚本每次运行都会重新构建。开发阶段较方便，但正式部署时会增加启动时间，并将构建和运行耦合在一起。

## 3. sender.launch.py

文件：

```text
sender/launch/sender.launch.py
```

配置文件按以下顺序查找：

```text
当前目录/config/sender_params.yaml
当前目录/sender/config/sender_params.yaml
install share/config/sender_params.yaml
```

通过 `sender/start.sh` 启动时，当前目录是 `sender/`，实际命中：

```text
sender/config/sender_params.yaml
```

launch 只创建一个进程：

```python
Node(
    package="sender",
    executable="sender_node",
    parameters=[str(config_file)],
)
```

launch 的其他工作只有：

- 设置 ROS 日志显示格式。
- 打印实际使用的配置文件路径。

launch 不保存业务参数默认值。

## 4. CMake 构建关系

`sender_node` 可执行文件由以下源文件组成：

```text
main.cpp
protocol/serial_frame.cpp
serial/serial_send_worker.cpp
serial/serial_writer.cpp
local_mqtt/local_mqtt_bridge.cpp
```

同时链接两个动态库：

```text
hik_camera_node
video_encoder_node
```

因此代码虽然按节点和模块拆分，运行时仍处于同一个进程。

## 5. main() 初始化过程

入口文件：

```text
sender/main.cpp
```

### 5.1 初始化 ROS 和信号处理

```cpp
rclcpp::init(argc, argv);
std::signal(SIGPIPE, SIG_IGN);
```

忽略 `SIGPIPE` 后，如果 local MQTT 子进程退出或 pipe 断开，C++ 写 pipe 时不会因为信号直接终止整个 sender 进程，而是由写调用返回错误。

随后启用 ROS 进程内通信：

```cpp
options.use_intra_process_comms(true);
```

这个选项的意图是减少同进程节点之间的消息复制。不过相机使用 `image_transport::CameraPublisher`，是否真正获得零拷贝收益还需要进一步验证。

### 5.2 创建 sender_runtime

```cpp
auto runtime_node = std::make_shared<rclcpp::Node>("sender_runtime", options);
```

该节点读取 sender 总体配置：

```text
enable_serial
enable_local_mqtt
local_mqtt_start_broker
local_mqtt_host
local_mqtt_port
local_mqtt_topic
```

参数读取函数没有业务默认值。YAML 缺少参数时会尽早报错，符合 config 是唯一配置入口的项目原则。

### 5.3 创建 HikCameraNode

```cpp
auto camera_node = std::make_shared<sniper::camera::HikCameraNode>(options);
```

构造函数立即执行：

```text
枚举 USB 海康相机
-> 没找到时每秒重新枚举
-> 创建并打开第一个相机
-> 查询图像尺寸
-> 创建 image_raw publisher
-> 读取并设置相机参数
-> MV_CC_StartGrabbing
-> 创建相机采集线程
```

相机采集线程不断执行：

```text
MV_CC_GetImageBuffer
-> Bayer 图像转 BGR
-> 填充 sensor_msgs::Image
-> 发布 /image_raw 和 CameraInfo
-> MV_CC_FreeImageBuffer
```

重要时序：如果启动时没有相机，`main()` 会一直阻塞在相机节点的构造函数中，编码器、串口和 local MQTT 都不会创建。

优化关注点：相机构造函数同时承担设备等待、初始化和线程启动，使启动过程难以分阶段控制和观测。

### 5.4 创建 VideoEncoderNode

```cpp
auto encoder_node = std::make_shared<sniper::encoder::VideoEncoderNode>(options);
```

构造函数依次执行：

```text
读取 video_encoder 全部参数
-> 校验并裁剪参数范围
-> 订阅 /image_raw
-> 创建 GStreamer pipeline
-> enable_display=true 时创建 OpenCV 显示线程
```

订阅回调关系：

```text
HikCameraNode 发布 /image_raw
-> ROS executor
-> VideoEncoderNode::image_callback()
```

此时 executor 尚未进入 `spin()`，但相机采集线程已经开始发布图像。这是节点构造阶段启动工作线程带来的时序交叉。

### 5.5 创建 SerialSendWorker

`main()` 先从编码器节点取得：

```text
serial_max_rate_hz
max_tx_delay_s
```

然后创建并启动发送器：

```cpp
SerialSendWorker serial_worker(std::move(send_config));
serial_worker.start();
```

`start()` 的执行过程：

```text
enable_serial=true
  -> SerialWriter::start()

enable_local_mqtt=true
  -> LocalMqttBridge::start()
  -> fork/exec Python local_mqtt/main.py

最后
  -> 创建 SerialSendWorker::run() 发送线程
```

### 5.6 连接编码器和发送器

```cpp
encoder_node->set_serial_stream_callback(
    [&serial_worker](const uint8_t *data, size_t size) {
        serial_worker.enqueue(data, size);
    });
```

这里不是 ROS topic，而是一个普通 C++ 回调：

```text
GStreamer appsink 输出 H264
-> VideoEncoderNode 调用 serial_stream_cb_
-> SerialSendWorker::enqueue()
-> 将 H264 字节追加到 stream_buffer_
```

`serial_worker` 是 `main()` 栈上的对象，lambda 按引用捕获它。当前生命周期是安全的，但安全性依赖 `main()` 中固定的停止和销毁顺序，后续重构时需要注意。

### 5.7 启动 ROS executor

```cpp
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(runtime_node);
executor.add_node(camera_node);
executor.add_node(encoder_node);
executor.spin();
```

`spin()` 是主线程的长期阻塞点。收到退出信号或调用 `rclcpp::shutdown()` 后返回，随后执行：

```text
serial_worker.stop()
-> 停止发送线程
-> 停止 local MQTT 子进程
-> 停止 SerialWriter
-> rclcpp::shutdown()
-> main() 返回
```

## 6. 运行时线程和进程模型

```text
sender_node 主进程
|
|- 主线程 / ROS MultiThreadedExecutor
|   `- 调用 VideoEncoderNode::image_callback()
|
|- 相机采集线程
|   `- GetImageBuffer -> Bayer转BGR -> publish(/image_raw)
|
|- GStreamer 内部线程
|   `- x264编码 -> appsink回调 -> enqueue(H264)
|
|- SerialSendWorker 发送线程
|   `- 取299B -> 加inner seq -> 构造309B帧 -> 输出
|
|- SerialWriter 串口监控线程
|   `- 扫描和维护串口热插拔
|
|- 可选 OpenCV 显示线程
|   `- sender 预览窗口
|
`- 可选 local_mqtt Python 子进程
    `- stdin读取309B帧 -> 校验 -> MQTT发布
```

## 7. 完整数据调用链

```text
HikCameraNode capture_thread_
-> camera_pub_.publish()
-> /image_raw
-> VideoEncoderNode::image_callback()
-> 图像裁剪、缩放和静态背景简化
-> gst_app_src_push_buffer()
-> GStreamer x264
-> appsink callback
-> SerialSendWorker::enqueue()
-> SerialSendWorker::run()
-> 每次取299B H264
-> 加1B inner seq
-> protocol::build_frame()
-> 构造309B RoboMaster 0x0310帧
-> SerialWriter / LocalMqttBridge
```

## 8. 第一轮优化关注点

目前从启动层可以记录以下候选项：

1. `start.sh` 每次启动都重新构建，部署时可能增加不必要的启动成本。
2. 相机构造函数阻塞等待硬件，并在构造过程中启动采集线程，启动生命周期不够清晰。
3. 相机线程在 executor 开始 `spin()` 前就可能发布图像，存在构造阶段和运行阶段交叉。
4. `use_intra_process_comms(true)` 是否对 `image_transport::CameraPublisher` 到普通 subscription 的路径真正减少复制，需要实测。
5. 图像经过相机缓存、ROS 消息、OpenCV 和 GStreamer 时可能存在多次大块内存复制，需要沿数据路径逐处确认。
6. 编码器通过引用捕获栈上的 `SerialSendWorker`，当前可用，但生命周期关系较隐式。

## 9. 后续阅读顺序

建议继续按数据流阅读：

1. `HikCameraNode`：采集、颜色转换、ROS 发布及内存复制。
2. `VideoEncoderNode::image_callback()`：预处理和帧率控制。
3. GStreamer pipeline：appsrc、x264enc、h264parse、appsink。
4. `SerialSendWorker`：buffer、backlog 裁剪、限速和分包。
5. `build_frame()`：0x0310 帧结构及 CRC。
6. `SerialWriter`：串口热插拔和完整帧写入。
7. `LocalMqttBridge`：fork/exec、pipe 和 Python MQTT 测试链路。
