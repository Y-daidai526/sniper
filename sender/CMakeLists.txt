cmake_minimum_required(VERSION 3.8)
project(sender)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED
  gstreamer-1.0>=1.4
  gstreamer-app-1.0>=1.4
)

set(MVS_ROOT /opt/MVS)
set(MVS_INCLUDE_DIR ${MVS_ROOT}/include)
set(MVS_LIB_DIR ${MVS_ROOT}/lib/64)

add_executable(camera_encoder_node
  camera_encoder_node/camera_encoder_node.cpp
  hik_camera_node/hik_camera_node.cpp
  video_encoder_node/video_encoder_node.cpp
)
target_compile_features(camera_encoder_node PUBLIC cxx_std_17)
target_include_directories(camera_encoder_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${GSTREAMER_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${MVS_INCLUDE_DIR}
)
target_compile_options(camera_encoder_node PRIVATE ${GSTREAMER_CFLAGS_OTHER})
target_link_directories(camera_encoder_node PRIVATE ${GSTREAMER_LIBRARY_DIRS})
target_link_libraries(camera_encoder_node PUBLIC
  ${sensor_msgs_TARGETS}
  camera_info_manager::camera_info_manager
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  ${std_msgs_TARGETS}
  ${OpenCV_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
  ${MVS_LIB_DIR}/libFormatConversion.so
  ${MVS_LIB_DIR}/libMediaProcess.so
  ${MVS_LIB_DIR}/libMvCameraControl.so
  ${MVS_LIB_DIR}/libMVRender.so
  ${MVS_LIB_DIR}/libMvUsb3vTL.so
)

add_executable(rm_frame_encoder_node
  rm_frame_encoder_node/rm_frame_encoder_node.cpp
  rm_frame_encoder_node/rm_frame.cpp
)
target_compile_features(rm_frame_encoder_node PUBLIC cxx_std_17)
target_include_directories(rm_frame_encoder_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(rm_frame_encoder_node PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

add_executable(serial_sender_node
  serial_sender_node/serial_sender_node.cpp
  serial_sender_node/serial_writer.cpp
)
target_compile_features(serial_sender_node PUBLIC cxx_std_17)
target_include_directories(serial_sender_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(serial_sender_node PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

install(TARGETS
  camera_encoder_node
  rm_frame_encoder_node
  serial_sender_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(mqtt_broker_node)
ament_python_install_package(mqtt_sender_node)

install(PROGRAMS
  mqtt_broker_node/mqtt_broker_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mqtt_broker_node
)

install(PROGRAMS
  mqtt_sender_node/mqtt_sender_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mqtt_sender_node
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
