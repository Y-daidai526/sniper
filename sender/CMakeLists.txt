cmake_minimum_required(VERSION 3.8)
project(sender)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# --- 依赖 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

# GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED
  gstreamer-1.0>=1.4
  gstreamer-app-1.0>=1.4
)

# MVS SDK
set(MVS_ROOT /opt/MVS)
set(MVS_INCLUDE_DIR ${MVS_ROOT}/include)
set(MVS_LIB_DIR ${MVS_ROOT}/lib/64)

# doorlock_sniper — for VideoPacket.msg
find_package(doorlock_sniper REQUIRED)

# --- Common include dirs ---
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${GSTREAMER_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${MVS_INCLUDE_DIR}
)

link_directories(${GSTREAMER_LIBRARY_DIRS} ${MVS_LIB_DIR})
add_definitions(${GSTREAMER_CFLAGS_OTHER})

# ============================================================
# Camera Node (ComposableNode shared lib)
# ============================================================
add_library(hik_camera_node SHARED
  camera/hik_camera_node.cpp
)

target_compile_features(hik_camera_node PUBLIC cxx_std_17)

ament_target_dependencies(hik_camera_node
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  camera_info_manager
  OpenCV
)

target_link_libraries(hik_camera_node
  ${MVS_LIB_DIR}/libFormatConversion.so
  ${MVS_LIB_DIR}/libMediaProcess.so
  ${MVS_LIB_DIR}/libMvCameraControl.so
  ${MVS_LIB_DIR}/libMVRender.so
  ${MVS_LIB_DIR}/libMvUsb3vTL.so
)

rclcpp_components_register_nodes(hik_camera_node "sniper::camera::HikCameraNode")

# ============================================================
# Encoder Node (ComposableNode shared lib)
# ============================================================
add_library(video_encoder_node SHARED
  encoder/video_encoder_node.cpp
)

target_compile_features(video_encoder_node PUBLIC cxx_std_17)

ament_target_dependencies(video_encoder_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  std_msgs
  doorlock_sniper
  OpenCV
)

target_link_libraries(video_encoder_node
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
)

rclcpp_components_register_nodes(video_encoder_node "sniper::encoder::VideoEncoderNode")

# ============================================================
# Sender Executable (standalone: encoder + serial + debug)
# ============================================================
add_executable(sender_node
  main.cpp
  protocol/serial_frame.cpp
  serial/serial_writer.cpp
  debug/debug_bridge.cpp
)

target_compile_features(sender_node PUBLIC cxx_std_17)

ament_target_dependencies(sender_node
  rclcpp
)

target_link_libraries(sender_node
  video_encoder_node
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# ============================================================
# Install
# ============================================================
install(TARGETS hik_camera_node video_encoder_node sender_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# MQTT bridge: ROS2 Python node (lib/sender/ 下才能被 ros2 run 找到)
install(PROGRAMS debug/mqtt_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY debug/proto/
  DESTINATION lib/${PROJECT_NAME}/proto
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY debug/
  DESTINATION share/${PROJECT_NAME}/debug
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
