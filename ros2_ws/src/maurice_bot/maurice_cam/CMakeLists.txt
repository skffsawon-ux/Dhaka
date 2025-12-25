cmake_minimum_required(VERSION 3.8)
project(maurice_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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

# =============================================================================
# COMPONENT LIBRARIES (for composition / intra-process communication)
# =============================================================================

# Main camera driver component library
add_library(main_camera_driver_component SHARED maurice_cam/main_camera_driver.cpp)
target_compile_definitions(main_camera_driver_component PRIVATE BUILDING_COMPONENT_LIBRARY)
target_include_directories(main_camera_driver_component PRIVATE ${TURBOJPEG_INCLUDE_DIRS})
target_link_libraries(main_camera_driver_component ${OpenCV_LIBS} ${TURBOJPEG_LIBRARIES})
ament_target_dependencies(main_camera_driver_component 
  rclcpp 
  rclcpp_components
  sensor_msgs
  image_transport
)
rclcpp_components_register_nodes(main_camera_driver_component "maurice_cam::MainCameraDriver")

# Arm camera driver component library
add_library(arm_camera_driver_component SHARED maurice_cam/arm_camera_driver.cpp)
target_compile_definitions(arm_camera_driver_component PRIVATE BUILDING_COMPONENT_LIBRARY)
target_link_libraries(arm_camera_driver_component ${OpenCV_LIBS})
ament_target_dependencies(arm_camera_driver_component 
  rclcpp 
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_nodes(arm_camera_driver_component "maurice_cam::ArmCameraDriver")

# WebRTC streamer component library
add_library(webrtc_streamer_component SHARED maurice_cam/webrtc_streamer.cpp)
target_compile_definitions(webrtc_streamer_component PRIVATE BUILDING_COMPONENT_LIBRARY)
target_include_directories(webrtc_streamer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GSTREAMER_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(webrtc_streamer_component
  ${GSTREAMER_LIBRARIES}
  ${OpenCV_LIBS}
)
if(nlohmann_json_FOUND)
  target_link_libraries(webrtc_streamer_component nlohmann_json::nlohmann_json)
elseif(NLOHMANN_JSON_FOUND)
  target_include_directories(webrtc_streamer_component PUBLIC ${NLOHMANN_JSON_INCLUDE_DIRS})
endif()
ament_target_dependencies(webrtc_streamer_component
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
)
rclcpp_components_register_nodes(webrtc_streamer_component "maurice_cam::WebRTCStreamer")

# =============================================================================
# STANDALONE EXECUTABLES (for running nodes individually)
# =============================================================================

# Create main camera driver executable
add_executable(main_camera_driver maurice_cam/main_camera_driver.cpp)
target_include_directories(main_camera_driver PRIVATE ${TURBOJPEG_INCLUDE_DIRS})
target_link_libraries(main_camera_driver ${OpenCV_LIBS} ${TURBOJPEG_LIBRARIES})
ament_target_dependencies(main_camera_driver 
  rclcpp 
  rclcpp_components
  sensor_msgs
  image_transport
)

# Create arm camera driver executable
add_executable(arm_camera_driver maurice_cam/arm_camera_driver.cpp)
target_link_libraries(arm_camera_driver ${OpenCV_LIBS})
ament_target_dependencies(arm_camera_driver 
  rclcpp 
  rclcpp_components
  sensor_msgs
)

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
  rclcpp_components
  std_msgs
  sensor_msgs
)

# =============================================================================
# INSTALLATION
# =============================================================================

# Install component libraries
install(TARGETS
  main_camera_driver_component
  arm_camera_driver_component
  webrtc_streamer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
