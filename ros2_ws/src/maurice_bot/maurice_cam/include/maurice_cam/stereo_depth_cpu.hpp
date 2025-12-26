#pragma once

#include <memory>
#include <string>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

namespace maurice_cam
{

/**
 * @brief CPU-based stereo depth estimator using OpenCV
 * 
 * Simple implementation for comparison with VPI-based GPU version.
 * Uses OpenCV's StereoSGBM algorithm on CPU.
 */
class StereoDepthCPU : public rclcpp::Node
{
public:
  explicit StereoDepthCPU(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StereoDepthCPU() = default;

private:
  std::filesystem::path findCalibrationConfigDir();
  bool loadCalibration(const std::filesystem::path& calib_path);
  void stereoImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  // ROS 2 publishers and subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr stereo_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;

  // Parameters
  std::string data_directory_;
  std::string stereo_topic_;
  std::string depth_topic_;
  std::string disparity_topic_;
  std::string frame_id_;
  int max_disparity_;
  int block_size_;
  bool publish_disparity_;
  int process_every_n_frames_;
  int stereo_width_;
  int stereo_height_;
  int image_width_;
  int image_height_;

  // Calibration
  cv::Mat K1_, D1_, K2_, D2_;
  cv::Mat R1_, R2_, P1_, P2_, Q_;
  cv::Mat map1_left_, map2_left_;
  cv::Mat map1_right_, map2_right_;
  double baseline_;
  double focal_length_;
  bool calibration_loaded_{false};

  // OpenCV stereo matcher
  cv::Ptr<cv::StereoSGBM> stereo_matcher_;

  // Stats
  int frame_count_{0};
  int input_frame_count_{0};
  rclcpp::Time last_stats_time_;
};

} // namespace maurice_cam

