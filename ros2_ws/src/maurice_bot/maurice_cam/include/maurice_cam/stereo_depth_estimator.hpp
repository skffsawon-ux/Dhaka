#pragma once

#include <memory>
#include <string>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <opencv2/opencv.hpp>

// VPI headers
#include <vpi/VPI.h>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/algo/Remap.h>
#include <vpi/algo/StereoDisparity.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/MedianFilter.h>
#include <vpi/algo/BilateralFilter.h>
#include <vpi/WarpMap.h>

namespace maurice_cam
{

/**
 * @brief Stereo Depth Estimator component node using NVIDIA VPI
 * 
 * This node subscribes to stereo image topic using zero-copy intra-process
 * communication, performs GPU-accelerated rectification, disparity and depth
 * estimation using NVIDIA VPI, and publishes the depth image.
 */
class StereoDepthEstimator : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options Node options for component composition
   */
  explicit StereoDepthEstimator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~StereoDepthEstimator();

private:
  /**
   * @brief Find calibration config directory based on robot_info.json
   * @return Path to calibration config directory
   */
  std::filesystem::path findCalibrationConfigDir();

  /**
   * @brief Load stereo calibration parameters from YAML file
   * @param calib_path Path to the calibration YAML file
   * @return true if successful, false otherwise
   */
  bool loadCalibration(const std::filesystem::path& calib_path);

  /**
   * @brief Initialize VPI resources (streams, payloads, images)
   * @return true if successful, false otherwise
   */
  bool initializeVPI();

  /**
   * @brief Create warp maps for stereo rectification
   * @return true if successful, false otherwise
   */
  bool createWarpMaps();

  /**
   * @brief Cleanup VPI resources
   */
  void cleanupVPI();

  /**
   * @brief Callback for stereo image subscription
   * @param msg Incoming stereo image message (zero-copy unique_ptr)
   */
  void stereoImageCallback(sensor_msgs::msg::Image::UniquePtr msg);

  /**
   * @brief Process stereo frame: rectify, compute disparity, compute depth
   * @param stereo_frame Input stereo frame (side-by-side left and right)
   * @param timestamp Timestamp for the output message
   */
  void processFrame(const cv::Mat& stereo_frame, const rclcpp::Time& timestamp);

  // ROS 2 publishers and subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr stereo_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  // Node parameters
  std::string data_directory_;
  std::string stereo_topic_;
  std::string depth_topic_;
  std::string disparity_topic_;
  std::string pointcloud_topic_;
  std::string frame_id_;
  int max_disparity_;
  bool publish_disparity_;
  bool publish_pointcloud_;
  int process_every_n_frames_;  // Process 1 out of every N frames
  int pointcloud_decimation_;   // Decimate point cloud (1=full, 2=half, 4=quarter)

  // Disparity filtering parameters
  bool enable_disparity_filter_;    // Enable disparity filtering pipeline
  int median_filter_size_;          // Median filter kernel size (0=disabled, 3/5/7)
  int bilateral_filter_size_;       // Bilateral filter kernel size (0=disabled, 3/5/7/9)
  float bilateral_sigma_space_;     // Bilateral spatial sigma
  float bilateral_sigma_color_;     // Bilateral intensity sigma

  // Quality parameters (to match OpenCV SGBM)
  int confidence_threshold_;        // Reject pixels with confidence below this (0-65535)
  int min_disparity_threshold_;     // Minimum disparity in Q10.5 (reject noise)
  int window_size_;                 // SGM window size

  // Image dimensions
  int stereo_width_;   // Full stereo image width (left + right)
  int stereo_height_;  // Full stereo image height
  int image_width_;    // Single camera image width
  int image_height_;   // Single camera image height

  // Calibration parameters (loaded from YAML)
  cv::Mat K1_, D1_;  // Left camera intrinsics and distortion
  cv::Mat K2_, D2_;  // Right camera intrinsics and distortion
  cv::Mat R_, T_;    // Rotation and translation between cameras
  cv::Mat R1_, R2_;  // Rectification transforms
  cv::Mat P1_, P2_;  // Projection matrices
  cv::Mat Q_;        // Disparity-to-depth mapping matrix
  double baseline_;  // Baseline distance (meters)
  double focal_length_; // Focal length in pixels (after rectification)

  // OpenCV rectification maps (used to build VPI warp maps)
  cv::Mat map1_left_, map2_left_;
  cv::Mat map1_right_, map2_right_;

  // VPI resources
  VPIStream vpi_stream_{nullptr};
  
  // VPI images
  VPIImage vpi_left_input_{nullptr};
  VPIImage vpi_right_input_{nullptr};
  VPIImage vpi_left_gray_{nullptr};
  VPIImage vpi_right_gray_{nullptr};
  VPIImage vpi_left_rectified_{nullptr};
  VPIImage vpi_right_rectified_{nullptr};
  VPIImage vpi_disparity_{nullptr};
  VPIImage vpi_disparity_filtered_{nullptr};  // For median/bilateral filtering
  VPIImage vpi_disparity_temp_{nullptr};      // Temp buffer for filter chain
  VPIImage vpi_confidence_{nullptr};

  // VPI payloads
  VPIPayload remap_left_payload_{nullptr};
  VPIPayload remap_right_payload_{nullptr};
  VPIPayload stereo_payload_{nullptr};

  // VPI warp maps
  VPIWarpMap warp_map_left_{};
  VPIWarpMap warp_map_right_{};
  bool warp_maps_allocated_{false};

  // Processing state
  bool vpi_initialized_{false};
  bool calibration_loaded_{false};

  // Frame statistics and rate control
  int frame_count_{0};
  int input_frame_count_{0};  // Total input frames received
  rclcpp::Time last_stats_time_;
};

} // namespace maurice_cam

