cmake_minimum_required(VERSION 3.8)
project(maurice_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find GStreamer (for webrtc_streamer)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TURBOJPEG REQUIRED libturbojpeg)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0)

# Find nlohmann_json (for webrtc_streamer)
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  pkg_check_modules(NLOHMANN_JSON nlohmann_json)
endif()

# Set Python version
set(PYTHON_VERSION "python3.10")

# Include directories
include_directories(include)

# Create main camera driver executable
add_executable(main_camera_driver maurice_cam/main_camera_driver.cpp)
ament_target_dependencies(main_camera_driver 
  rclcpp 
  sensor_msgs
  cv_bridge
  image_transport
)
target_include_directories(main_camera_driver PRIVATE ${TURBOJPEG_INCLUDE_DIRS})
target_link_libraries(main_camera_driver ${OpenCV_LIBS} ${TURBOJPEG_LIBRARIES})

# Create arm camera driver executable
add_executable(arm_camera_driver maurice_cam/arm_camera_driver.cpp)
ament_target_dependencies(arm_camera_driver 
  rclcpp 
  sensor_msgs
  cv_bridge
)
target_link_libraries(arm_camera_driver ${OpenCV_LIBS})

# Create WebRTC streamer executable
add_executable(webrtc_streamer_node maurice_cam/webrtc_streamer.cpp)
target_include_directories(webrtc_streamer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GSTREAMER_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(webrtc_streamer_node
  ${GSTREAMER_LIBRARIES}
  ${OpenCV_LIBS}
)
if(nlohmann_json_FOUND)
  target_link_libraries(webrtc_streamer_node nlohmann_json::nlohmann_json)
elseif(NLOHMANN_JSON_FOUND)
  target_include_directories(webrtc_streamer_node PUBLIC ${NLOHMANN_JSON_INCLUDE_DIRS})
endif()
ament_target_dependencies(webrtc_streamer_node
  rclcpp
  std_msgs
  sensor_msgs
)

# Install C++ executables
install(TARGETS
  main_camera_driver
  arm_camera_driver
  webrtc_streamer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python module to site-packages (makes it importable as a Python package)
install(
  DIRECTORY maurice_cam/
  DESTINATION lib/${PYTHON_VERSION}/site-packages/maurice_cam
  PATTERN "*.cpp" EXCLUDE
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
