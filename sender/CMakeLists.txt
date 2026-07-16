cmake_minimum_required(VERSION 3.8)
project(sender)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
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

add_library(hik_camera_node SHARED
  camera/hik_camera_node.cpp
)
target_compile_features(hik_camera_node PUBLIC cxx_std_17)
target_include_directories(hik_camera_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${MVS_INCLUDE_DIR}
)
target_link_libraries(hik_camera_node PUBLIC
  ${sensor_msgs_TARGETS}
  camera_info_manager::camera_info_manager
  image_transport::image_transport
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  sensor_msgs::sensor_msgs_library
  ${OpenCV_LIBRARIES}
  ${MVS_LIB_DIR}/libFormatConversion.so
  ${MVS_LIB_DIR}/libMediaProcess.so
  ${MVS_LIB_DIR}/libMvCameraControl.so
  ${MVS_LIB_DIR}/libMVRender.so
  ${MVS_LIB_DIR}/libMvUsb3vTL.so
)
rclcpp_components_register_nodes(hik_camera_node "sniper::camera::HikCameraNode")

add_library(video_encoder_node SHARED
  encoder/video_encoder_node.cpp
)
target_compile_features(video_encoder_node PUBLIC cxx_std_17)
target_include_directories(video_encoder_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${GSTREAMER_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_options(video_encoder_node PRIVATE ${GSTREAMER_CFLAGS_OTHER})
target_link_directories(video_encoder_node PRIVATE ${GSTREAMER_LIBRARY_DIRS})
target_link_libraries(video_encoder_node PUBLIC
  ${sensor_msgs_TARGETS}
  cv_bridge::cv_bridge
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  sensor_msgs::sensor_msgs_library
  ${OpenCV_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
)
rclcpp_components_register_nodes(video_encoder_node "sniper::encoder::VideoEncoderNode")

add_executable(sender_node
  main.cpp
  protocol/serial_frame.cpp
  serial/serial_send_worker.cpp
  serial/serial_writer.cpp
  local_test/local_test_bridge.cpp
)
target_compile_features(sender_node PUBLIC cxx_std_17)
target_include_directories(sender_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(sender_node PRIVATE
  hik_camera_node
  video_encoder_node
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS hik_camera_node video_encoder_node sender_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY local_test/
  DESTINATION share/${PROJECT_NAME}/local_test
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
