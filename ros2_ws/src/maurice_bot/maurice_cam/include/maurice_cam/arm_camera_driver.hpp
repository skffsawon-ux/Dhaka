#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <string.h>
#include <chrono>
#include <thread>
#include <sys/select.h>
#include <sys/stat.h>

namespace maurice_cam
{

/**
 * @brief V4L2-based arm camera driver node for Maurice robot
 * 
 * This node provides a direct V4L2 interface to capture frames from the arm-mounted
 * Arducam USB camera and publishes both raw and compressed images.
 */
class ArmCameraDriver : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options Node options for component composition
   */
  explicit ArmCameraDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~ArmCameraDriver();

private:
  /**
   * @brief Initialize camera with V4L2
   * @return true if successful, false otherwise
   */
  bool init_camera();

  /**
   * @brief Handle stream errors and attempt recovery
   */
  void handle_stream_error();

  /**
   * @brief Capture frame and publish
   */
  void capture_and_publish();

  // ROS 2 publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Camera file descriptor
  int fd_ = -1;
  
  // Camera parameters
  std::string device_path_;
  int width_;
  int height_;
  int fps_;

  // V4L2 buffers
  struct Buffer {
    void* start;
    size_t length;
  } buffers_[4];

  // Error counter for retry logic
  int invalid_buffer_index_count_ = 0;
  
  static const int MAX_ERROR_COUNT = 3;  // Allow 3 consecutive errors before reconnecting
};

} // namespace maurice_cam

