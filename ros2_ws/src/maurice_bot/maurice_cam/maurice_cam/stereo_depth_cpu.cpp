#include "maurice_cam/stereo_depth_cpu.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <chrono>

namespace maurice_cam
{

StereoDepthCPU::StereoDepthCPU(const rclcpp::NodeOptions & options)
: Node("stereo_depth_cpu", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("data_directory", "/home/jetson1/innate-os/data");
  this->declare_parameter<std::string>("stereo_topic", "/mars/main_camera/stereo");
  this->declare_parameter<std::string>("depth_topic", "/mars/main_camera/depth_cpu");
  this->declare_parameter<std::string>("disparity_topic", "/mars/main_camera/disparity_cpu");
  this->declare_parameter<std::string>("frame_id", "camera_optical_frame");
  this->declare_parameter<int>("max_disparity", 64);
  this->declare_parameter<int>("block_size", 5);
  this->declare_parameter<bool>("publish_disparity", true);
  this->declare_parameter<int>("process_every_n_frames", 5);
  this->declare_parameter<int>("stereo_width", 1280);
  this->declare_parameter<int>("stereo_height", 480);

  // Get parameters
  data_directory_ = this->get_parameter("data_directory").as_string();
  stereo_topic_ = this->get_parameter("stereo_topic").as_string();
  depth_topic_ = this->get_parameter("depth_topic").as_string();
  disparity_topic_ = this->get_parameter("disparity_topic").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  max_disparity_ = this->get_parameter("max_disparity").as_int();
  block_size_ = this->get_parameter("block_size").as_int();
  publish_disparity_ = this->get_parameter("publish_disparity").as_bool();
  process_every_n_frames_ = this->get_parameter("process_every_n_frames").as_int();
  if (process_every_n_frames_ < 1) process_every_n_frames_ = 1;
  stereo_width_ = this->get_parameter("stereo_width").as_int();
  stereo_height_ = this->get_parameter("stereo_height").as_int();

  image_width_ = stereo_width_ / 2;
  image_height_ = stereo_height_;

  RCLCPP_INFO(this->get_logger(), "=== Stereo Depth CPU (OpenCV) ===");
  RCLCPP_INFO(this->get_logger(), "Stereo topic: %s", stereo_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Depth topic: %s", depth_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Image size: %dx%d", image_width_, image_height_);
  RCLCPP_INFO(this->get_logger(), "Max disparity: %d, block size: %d", max_disparity_, block_size_);
  RCLCPP_INFO(this->get_logger(), "Process rate: 1/%d frames", process_every_n_frames_);

  // Load calibration
  try {
    auto calib_dir = findCalibrationConfigDir();
    auto calib_file = calib_dir / "stereo_calib.yaml";
    if (!loadCalibration(calib_file)) {
      throw std::runtime_error("Failed to load calibration");
    }
    calibration_loaded_ = true;
    RCLCPP_INFO(this->get_logger(), "Loaded calibration from: %s", calib_file.string().c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Calibration error: %s", e.what());
    throw;
  }

  // Create OpenCV StereoSGBM matcher
  // max_disparity must be divisible by 16
  int num_disparities = ((max_disparity_ + 15) / 16) * 16;
  
  stereo_matcher_ = cv::StereoSGBM::create(
    0,                    // minDisparity
    num_disparities,      // numDisparities (must be divisible by 16)
    block_size_,          // blockSize (odd number, 3-11)
    8 * block_size_ * block_size_,   // P1 (smoothness penalty)
    32 * block_size_ * block_size_,  // P2 (larger smoothness penalty)
    1,                    // disp12MaxDiff
    63,                   // preFilterCap
    10,                   // uniquenessRatio
    100,                  // speckleWindowSize
    1,                    // speckleRange
    cv::StereoSGBM::MODE_SGBM_3WAY  // mode (fastest SGBM variant)
  );

  RCLCPP_INFO(this->get_logger(), "Created StereoSGBM matcher (numDisparities=%d)", num_disparities);

  // Create publishers
  depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    depth_topic_,
    rclcpp::SensorDataQoS().reliability(rclcpp::ReliabilityPolicy::BestEffort)
  );

  if (publish_disparity_) {
    disparity_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      disparity_topic_,
      rclcpp::SensorDataQoS().reliability(rclcpp::ReliabilityPolicy::BestEffort)
    );
  }

  // Create subscription
  stereo_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    stereo_topic_,
    rclcpp::SensorDataQoS().reliability(rclcpp::ReliabilityPolicy::BestEffort),
    std::bind(&StereoDepthCPU::stereoImageCallback, this, std::placeholders::_1)
  );

  last_stats_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "Stereo Depth CPU initialized");
}

std::filesystem::path StereoDepthCPU::findCalibrationConfigDir()
{
  std::filesystem::path data_path(data_directory_);
  std::filesystem::path robot_info_path = data_path / "robot_info.json";

  std::string robot_model;
  if (std::filesystem::exists(robot_info_path)) {
    try {
      std::ifstream file(robot_info_path);
      nlohmann::json robot_info;
      file >> robot_info;
      if (robot_info.contains("model")) {
        robot_model = robot_info["model"].get<std::string>();
      }
    } catch (...) {}
  }

  for (const auto& entry : std::filesystem::directory_iterator(data_path)) {
    if (entry.is_directory()) {
      std::string dirname = entry.path().filename().string();
      if (dirname.find("calibration_config") != std::string::npos) {
        if (!robot_model.empty() && dirname.find(robot_model) != std::string::npos) {
          return entry.path();
        }
        if (robot_model.empty()) {
          return entry.path();
        }
      }
    }
  }

  // Fallback
  for (const auto& entry : std::filesystem::directory_iterator(data_path)) {
    if (entry.is_directory()) {
      std::string dirname = entry.path().filename().string();
      if (dirname.find("calibration_config") != std::string::npos) {
        return entry.path();
      }
    }
  }

  throw std::runtime_error("No calibration_config directory found");
}

bool StereoDepthCPU::loadCalibration(const std::filesystem::path& calib_path)
{
  if (!std::filesystem::exists(calib_path)) {
    RCLCPP_ERROR(this->get_logger(), "Calibration file not found: %s", calib_path.string().c_str());
    return false;
  }

  cv::FileStorage fs(calib_path.string(), cv::FileStorage::READ);
  if (!fs.isOpened()) {
    return false;
  }

  fs["K1"] >> K1_;
  fs["D1"] >> D1_;
  fs["K2"] >> K2_;
  fs["D2"] >> D2_;
  fs["R1"] >> R1_;
  fs["R2"] >> R2_;
  fs["P1"] >> P1_;
  fs["P2"] >> P2_;
  fs["Q"] >> Q_;

  int calib_width, calib_height;
  fs["image_width"] >> calib_width;
  fs["image_height"] >> calib_height;
  fs.release();

  if (K1_.empty() || P1_.empty() || Q_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Missing calibration matrices");
    return false;
  }

  // Scale if needed
  double scale_x = static_cast<double>(image_width_) / calib_width;
  double scale_y = static_cast<double>(image_height_) / calib_height;

  if (calib_width != image_width_ || calib_height != image_height_) {
    RCLCPP_INFO(this->get_logger(), "Scaling calibration from %dx%d to %dx%d",
                calib_width, calib_height, image_width_, image_height_);

    K1_.at<double>(0, 0) *= scale_x;
    K1_.at<double>(1, 1) *= scale_y;
    K1_.at<double>(0, 2) *= scale_x;
    K1_.at<double>(1, 2) *= scale_y;

    K2_.at<double>(0, 0) *= scale_x;
    K2_.at<double>(1, 1) *= scale_y;
    K2_.at<double>(0, 2) *= scale_x;
    K2_.at<double>(1, 2) *= scale_y;

    P1_.at<double>(0, 0) *= scale_x;
    P1_.at<double>(1, 1) *= scale_y;
    P1_.at<double>(0, 2) *= scale_x;
    P1_.at<double>(1, 2) *= scale_y;

    P2_.at<double>(0, 0) *= scale_x;
    P2_.at<double>(1, 1) *= scale_y;
    P2_.at<double>(0, 2) *= scale_x;
    P2_.at<double>(1, 2) *= scale_y;
    P2_.at<double>(0, 3) *= scale_x;

    Q_.at<double>(0, 3) *= scale_x;
    Q_.at<double>(1, 3) *= scale_y;
    Q_.at<double>(2, 3) *= scale_x;
  }

  // Extract baseline and focal length
  focal_length_ = Q_.at<double>(2, 3);
  double neg_inv_tx = Q_.at<double>(3, 2);
  baseline_ = std::abs(1.0 / neg_inv_tx);

  RCLCPP_INFO(this->get_logger(), "Stereo: focal=%.2f px, baseline=%.4f m",
              focal_length_, baseline_);

  // Compute rectification maps
  cv::initUndistortRectifyMap(K1_, D1_, R1_, P1_,
                               cv::Size(image_width_, image_height_),
                               CV_32FC1, map1_left_, map2_left_);
  cv::initUndistortRectifyMap(K2_, D2_, R2_, P2_,
                               cv::Size(image_width_, image_height_),
                               CV_32FC1, map1_right_, map2_right_);

  return true;
}

void StereoDepthCPU::stereoImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!calibration_loaded_) return;

  // Rate control
  input_frame_count_++;
  if ((input_frame_count_ % process_every_n_frames_) != 0) return;

  // Validate dimensions
  if (static_cast<int>(msg->width) != stereo_width_ ||
      static_cast<int>(msg->height) != stereo_height_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Size mismatch: %dx%d vs %dx%d",
                         msg->width, msg->height, stereo_width_, stereo_height_);
    return;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  // Wrap as cv::Mat
  cv::Mat stereo_frame;
  if (msg->encoding == "bgr8") {
    stereo_frame = cv::Mat(msg->height, msg->width, CV_8UC3, 
                           const_cast<uint8_t*>(msg->data.data()), msg->step);
  } else if (msg->encoding == "rgb8") {
    cv::Mat rgb(msg->height, msg->width, CV_8UC3,
                const_cast<uint8_t*>(msg->data.data()), msg->step);
    cv::cvtColor(rgb, stereo_frame, cv::COLOR_RGB2BGR);
  } else if (msg->encoding == "mono8") {
    stereo_frame = cv::Mat(msg->height, msg->width, CV_8UC1,
                           const_cast<uint8_t*>(msg->data.data()), msg->step);
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Unsupported encoding: %s", msg->encoding.c_str());
    return;
  }

  // Split stereo frame
  cv::Mat left_img = stereo_frame(cv::Rect(0, 0, image_width_, image_height_));
  cv::Mat right_img = stereo_frame(cv::Rect(image_width_, 0, image_width_, image_height_));

  // Convert to grayscale if needed
  cv::Mat left_gray, right_gray;
  if (stereo_frame.channels() == 3) {
    cv::cvtColor(left_img, left_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_img, right_gray, cv::COLOR_BGR2GRAY);
  } else {
    left_gray = left_img;
    right_gray = right_img;
  }

  // Rectify images
  cv::Mat left_rect, right_rect;
  cv::remap(left_gray, left_rect, map1_left_, map2_left_, cv::INTER_LINEAR);
  cv::remap(right_gray, right_rect, map1_right_, map2_right_, cv::INTER_LINEAR);

  // Compute disparity using StereoSGBM
  cv::Mat disparity_s16;
  stereo_matcher_->compute(left_rect, right_rect, disparity_s16);

  // disparity_s16 is CV_16S with values scaled by 16 (fixed point)
  // Convert to depth (16UC1 in millimeters)
  cv::Mat depth_mm(image_height_, image_width_, CV_16UC1);
  const float bf_scaled_mm = static_cast<float>(baseline_ * focal_length_ * 16.0 * 1000.0);

  for (int y = 0; y < image_height_; y++) {
    const int16_t* disp_row = disparity_s16.ptr<int16_t>(y);
    uint16_t* depth_row = depth_mm.ptr<uint16_t>(y);
    for (int x = 0; x < image_width_; x++) {
      int16_t d = disp_row[x];
      if (d > 0) {
        float depth = bf_scaled_mm / static_cast<float>(d);
        depth_row[x] = static_cast<uint16_t>(std::clamp(depth, 1.0f, 65535.0f));
      } else {
        depth_row[x] = 0;
      }
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  double process_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

  // Publish depth
  auto depth_msg = std::make_unique<sensor_msgs::msg::Image>();
  depth_msg->header.stamp = msg->header.stamp;
  depth_msg->header.frame_id = frame_id_;
  depth_msg->height = image_height_;
  depth_msg->width = image_width_;
  depth_msg->encoding = "16UC1";
  depth_msg->is_bigendian = false;
  depth_msg->step = image_width_ * sizeof(uint16_t);
  depth_msg->data.resize(depth_msg->height * depth_msg->step);
  memcpy(depth_msg->data.data(), depth_mm.data, depth_msg->data.size());
  depth_pub_->publish(std::move(depth_msg));

  // Publish disparity visualization
  if (publish_disparity_ && disparity_pub_) {
    cv::Mat disp_vis;
    double min_val, max_val;
    cv::minMaxLoc(disparity_s16, &min_val, &max_val);
    disparity_s16.convertTo(disp_vis, CV_8U, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));

    auto disp_msg = std::make_unique<sensor_msgs::msg::Image>();
    disp_msg->header.stamp = msg->header.stamp;
    disp_msg->header.frame_id = frame_id_;
    disp_msg->height = image_height_;
    disp_msg->width = image_width_;
    disp_msg->encoding = "mono8";
    disp_msg->is_bigendian = false;
    disp_msg->step = image_width_;
    disp_msg->data.resize(disp_msg->height * disp_msg->step);
    memcpy(disp_msg->data.data(), disp_vis.data, disp_msg->data.size());
    disparity_pub_->publish(std::move(disp_msg));
  }

  frame_count_++;

  // Print stats every 20 frames
  if (frame_count_ % 20 == 0) {
    auto now = this->now();
    double elapsed = (now - last_stats_time_).seconds();
    double fps = 20.0 / elapsed;
    RCLCPP_INFO(this->get_logger(), "CPU Depth: %.1f FPS, %.1f ms/frame, %d frames",
                fps, process_ms, frame_count_);
    last_stats_time_ = now;
  }
}

} // namespace maurice_cam

// Main function for standalone execution
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<maurice_cam::StereoDepthCPU>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}

