#include "maurice_cam/stereo_depth_estimator.hpp"
#include <nlohmann/json.hpp>
#include <fstream>

using namespace std::chrono_literals;

// Macro to check VPI status and log errors
#define CHECK_VPI_STATUS(status, msg) \
  if ((status) != VPI_SUCCESS) { \
    char vpi_err[256]; \
    vpiGetLastStatusMessage(vpi_err, sizeof(vpi_err)); \
    RCLCPP_ERROR(this->get_logger(), "%s: %s", msg, vpi_err); \
    return false; \
  }

namespace maurice_cam
{

StereoDepthEstimator::StereoDepthEstimator(const rclcpp::NodeOptions & options)
: Node("stereo_depth_estimator", options)
{
  // Declare parameters with defaults
  this->declare_parameter<std::string>("data_directory", "/home/jetson1/innate-os/data");
  this->declare_parameter<std::string>("stereo_topic", "/mars/main_camera/stereo");
  this->declare_parameter<std::string>("depth_topic", "/mars/main_camera/depth");
  this->declare_parameter<std::string>("disparity_topic", "/mars/main_camera/disparity");
  this->declare_parameter<std::string>("frame_id", "camera_optical_frame");
  this->declare_parameter<int>("max_disparity", 64);
  this->declare_parameter<bool>("publish_disparity", false);
  this->declare_parameter<int>("stereo_width", 1280);  // Process at 1280x480 (640x480 per camera)
  this->declare_parameter<int>("stereo_height", 480);
  this->declare_parameter<int>("process_every_n_frames", 1);  // 1 = process every frame
  this->declare_parameter<bool>("publish_pointcloud", true);
  this->declare_parameter<std::string>("pointcloud_topic", "/mars/main_camera/points");
  this->declare_parameter<int>("pointcloud_decimation", 2);  // 2 = half resolution point cloud
  
  // Disparity filtering parameters
  this->declare_parameter<bool>("enable_disparity_filter", true);
  this->declare_parameter<int>("median_filter_size", 5);      // 0=disabled, 3/5/7
  this->declare_parameter<int>("bilateral_filter_size", 5);   // 0=disabled, 3/5/7/9
  this->declare_parameter<double>("bilateral_sigma_space", 1.5);
  this->declare_parameter<double>("bilateral_sigma_color", 50.0);
  
  // Quality parameters (to match OpenCV SGBM quality)
  this->declare_parameter<int>("confidence_threshold", 32768);  // 0-65535, reject below this
  this->declare_parameter<int>("min_disparity_threshold", 16);  // Q11.4: 16 = 1 pixel disparity
  this->declare_parameter<int>("window_size", 5);               // SGM window size (3-11, odd)

  // Get parameter values
  data_directory_ = this->get_parameter("data_directory").as_string();
  stereo_topic_ = this->get_parameter("stereo_topic").as_string();
  depth_topic_ = this->get_parameter("depth_topic").as_string();
  disparity_topic_ = this->get_parameter("disparity_topic").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  max_disparity_ = this->get_parameter("max_disparity").as_int();
  publish_disparity_ = this->get_parameter("publish_disparity").as_bool();
  stereo_width_ = this->get_parameter("stereo_width").as_int();
  stereo_height_ = this->get_parameter("stereo_height").as_int();
  process_every_n_frames_ = this->get_parameter("process_every_n_frames").as_int();
  if (process_every_n_frames_ < 1) process_every_n_frames_ = 1;
  publish_pointcloud_ = this->get_parameter("publish_pointcloud").as_bool();
  pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
  pointcloud_decimation_ = this->get_parameter("pointcloud_decimation").as_int();
  if (pointcloud_decimation_ < 1) pointcloud_decimation_ = 1;
  
  enable_disparity_filter_ = this->get_parameter("enable_disparity_filter").as_bool();
  median_filter_size_ = this->get_parameter("median_filter_size").as_int();
  bilateral_filter_size_ = this->get_parameter("bilateral_filter_size").as_int();
  bilateral_sigma_space_ = static_cast<float>(this->get_parameter("bilateral_sigma_space").as_double());
  bilateral_sigma_color_ = static_cast<float>(this->get_parameter("bilateral_sigma_color").as_double());

  // Calculate single image dimensions
  image_width_ = stereo_width_ / 2;
  image_height_ = stereo_height_;

  RCLCPP_INFO(this->get_logger(), "=== Maurice Stereo Depth Estimator ===");
  RCLCPP_INFO(this->get_logger(), "Data directory: %s", data_directory_.c_str());
  RCLCPP_INFO(this->get_logger(), "Stereo topic: %s", stereo_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Depth topic: %s", depth_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Stereo dimensions: %dx%d", stereo_width_, stereo_height_);
  RCLCPP_INFO(this->get_logger(), "Single image: %dx%d", image_width_, image_height_);
  RCLCPP_INFO(this->get_logger(), "Max disparity: %d", max_disparity_);
  if (process_every_n_frames_ > 1) {
    RCLCPP_INFO(this->get_logger(), "Process rate: 1/%d frames", process_every_n_frames_);
  }
  if (enable_disparity_filter_) {
    RCLCPP_INFO(this->get_logger(), "Disparity filtering: median=%d, bilateral=%d (sigma: %.1f, %.1f)",
                median_filter_size_, bilateral_filter_size_, 
                bilateral_sigma_space_, bilateral_sigma_color_);
  }

  // Find calibration config directory and load calibration
  try {
    auto calib_dir = findCalibrationConfigDir();
    auto calib_file = calib_dir / "stereo_calib.yaml";
    
    if (!loadCalibration(calib_file)) {
      throw std::runtime_error("Failed to load calibration from: " + calib_file.string());
    }
    calibration_loaded_ = true;
    RCLCPP_INFO(this->get_logger(), "Loaded calibration from: %s", calib_file.string().c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Calibration error: %s", e.what());
    throw;
  }

  // Initialize VPI
  if (!initializeVPI()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize VPI");
    throw std::runtime_error("VPI initialization failed");
  }
  vpi_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "VPI initialized successfully");

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

  if (publish_pointcloud_) {
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_,
      rclcpp::SensorDataQoS().reliability(rclcpp::ReliabilityPolicy::BestEffort)
    );
    RCLCPP_INFO(this->get_logger(), "Point cloud enabled: %s (decimation: %d)",
                pointcloud_topic_.c_str(), pointcloud_decimation_);
  }

  // Create subscription with zero-copy intra-process communication
  // Using unique_ptr callback for zero-copy transfer
  stereo_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    stereo_topic_,
    rclcpp::SensorDataQoS().reliability(rclcpp::ReliabilityPolicy::BestEffort),
    [this](sensor_msgs::msg::Image::UniquePtr msg) {
      this->stereoImageCallback(std::move(msg));
    }
  );

  last_stats_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "Stereo Depth Estimator initialized successfully");
}

StereoDepthEstimator::~StereoDepthEstimator()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Stereo Depth Estimator...");
  cleanupVPI();
  RCLCPP_INFO(this->get_logger(), "Stereo Depth Estimator shutdown complete");
}

std::filesystem::path StereoDepthEstimator::findCalibrationConfigDir()
{
  std::filesystem::path data_path(data_directory_);
  std::filesystem::path robot_info_path = data_path / "robot_info.json";

  // Try to read robot_info.json to get the robot model
  std::string robot_model;
  if (std::filesystem::exists(robot_info_path)) {
    try {
      std::ifstream file(robot_info_path);
      nlohmann::json robot_info;
      file >> robot_info;
      
      if (robot_info.contains("model")) {
        robot_model = robot_info["model"].get<std::string>();
        RCLCPP_INFO(this->get_logger(), "Found robot model: %s", robot_model.c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Could not parse robot_info.json: %s", e.what());
    }
  }

  // Look for calibration_config_* directories
  for (const auto& entry : std::filesystem::directory_iterator(data_path)) {
    if (entry.is_directory()) {
      std::string dirname = entry.path().filename().string();
      if (dirname.find("calibration_config") != std::string::npos) {
        // If we have a robot model, prefer matching directory
        if (!robot_model.empty() && dirname.find(robot_model) != std::string::npos) {
          RCLCPP_INFO(this->get_logger(), "Found matching calibration dir: %s", dirname.c_str());
          return entry.path();
        }
        // Otherwise use the first calibration_config directory found
        if (robot_model.empty()) {
          RCLCPP_INFO(this->get_logger(), "Using calibration dir: %s", dirname.c_str());
          return entry.path();
        }
      }
    }
  }

  // If we have a robot model but didn't find exact match, use any calibration_config dir
  for (const auto& entry : std::filesystem::directory_iterator(data_path)) {
    if (entry.is_directory()) {
      std::string dirname = entry.path().filename().string();
      if (dirname.find("calibration_config") != std::string::npos) {
        RCLCPP_WARN(this->get_logger(), "No exact match for model %s, using: %s", 
                    robot_model.c_str(), dirname.c_str());
        return entry.path();
      }
    }
  }

  throw std::runtime_error("No calibration_config directory found in: " + data_directory_);
}

bool StereoDepthEstimator::loadCalibration(const std::filesystem::path& calib_path)
{
  if (!std::filesystem::exists(calib_path)) {
    RCLCPP_ERROR(this->get_logger(), "Calibration file not found: %s", calib_path.string().c_str());
    return false;
  }

  cv::FileStorage fs(calib_path.string(), cv::FileStorage::READ);
  if (!fs.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open calibration file: %s", calib_path.string().c_str());
    return false;
  }

  // Read calibration parameters
  fs["K1"] >> K1_;
  fs["D1"] >> D1_;
  fs["K2"] >> K2_;
  fs["D2"] >> D2_;
  fs["R"] >> R_;
  fs["T"] >> T_;
  fs["R1"] >> R1_;
  fs["R2"] >> R2_;
  fs["P1"] >> P1_;
  fs["P2"] >> P2_;
  fs["Q"] >> Q_;

  // Read image dimensions from calibration file
  int calib_width, calib_height;
  fs["image_width"] >> calib_width;
  fs["image_height"] >> calib_height;

  fs.release();

  // Validate required matrices
  if (K1_.empty() || K2_.empty() || R1_.empty() || R2_.empty() || 
      P1_.empty() || P2_.empty() || Q_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Missing required calibration matrices");
    return false;
  }

  // Verify calibration dimensions match runtime dimensions
  if (calib_width != image_width_ || calib_height != image_height_) {
    RCLCPP_ERROR(this->get_logger(), 
                 "Calibration dimensions (%dx%d) do not match runtime dimensions (%dx%d). "
                 "Please recalibrate at the correct resolution.",
                 calib_width, calib_height, image_width_, image_height_);
    return false;
  }

  // Extract baseline and focal length from Q matrix
  // Q matrix from cv::stereoRectify is structured as:
  // [ 1  0  0       -cx          ]
  // [ 0  1  0       -cy          ]
  // [ 0  0  0        f           ]   <- Q[2,3] = focal length
  // [ 0  0  -1/Tx  (cx-cx')/Tx   ]   <- Q[3,2] = -1/Tx where Tx = baseline
  //
  // Note: Tx is the x-translation between cameras (baseline with sign)
  // Q[3,2] = -1/Tx, so Tx = -1/Q[3,2], and baseline = |Tx|
  focal_length_ = Q_.at<double>(2, 3);
  double neg_inv_tx = Q_.at<double>(3, 2);  // This is -1/Tx
  if (std::abs(neg_inv_tx) > 1e-6) {
    baseline_ = std::abs(1.0 / neg_inv_tx);
  } else {
    // Fallback: extract baseline from T vector directly
    baseline_ = std::abs(T_.at<double>(0, 0));
    RCLCPP_WARN(this->get_logger(), "Q[3,2] near zero, using T vector for baseline");
  }

  RCLCPP_INFO(this->get_logger(), "Stereo parameters: focal=%.2f px, baseline=%.4f m (%.1f mm)",
              focal_length_, baseline_, baseline_ * 1000.0);

  // Compute OpenCV rectification maps
  cv::initUndistortRectifyMap(K1_, D1_, R1_, P1_, 
                               cv::Size(image_width_, image_height_),
                               CV_32FC1, map1_left_, map2_left_);
  cv::initUndistortRectifyMap(K2_, D2_, R2_, P2_,
                               cv::Size(image_width_, image_height_),
                               CV_32FC1, map1_right_, map2_right_);
  RCLCPP_INFO(this->get_logger(), "Rectification maps computed (%dx%d)", image_width_, image_height_);

  return true;
}

bool StereoDepthEstimator::initializeVPI()
{
  VPIStatus status;

  // Create VPI stream with CUDA backend
  status = vpiStreamCreate(VPI_BACKEND_CUDA, &vpi_stream_);
  CHECK_VPI_STATUS(status, "Failed to create VPI stream");

  // Create warp maps for rectification
  if (!createWarpMaps()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create warp maps");
    return false;
  }

  // Create remap payloads for left and right images
  status = vpiCreateRemap(VPI_BACKEND_CUDA, &warp_map_left_, &remap_left_payload_);
  CHECK_VPI_STATUS(status, "Failed to create left remap payload");

  status = vpiCreateRemap(VPI_BACKEND_CUDA, &warp_map_right_, &remap_right_payload_);
  CHECK_VPI_STATUS(status, "Failed to create right remap payload");

  // Create VPI images for processing
  // Input images (grayscale)
  status = vpiImageCreate(image_width_, image_height_, VPI_IMAGE_FORMAT_U8,
                          VPI_BACKEND_CUDA, &vpi_left_gray_);
  CHECK_VPI_STATUS(status, "Failed to create left gray image");

  status = vpiImageCreate(image_width_, image_height_, VPI_IMAGE_FORMAT_U8,
                          VPI_BACKEND_CUDA, &vpi_right_gray_);
  CHECK_VPI_STATUS(status, "Failed to create right gray image");

  // Rectified images
  status = vpiImageCreate(image_width_, image_height_, VPI_IMAGE_FORMAT_U8,
                          VPI_BACKEND_CUDA, &vpi_left_rectified_);
  CHECK_VPI_STATUS(status, "Failed to create left rectified image");

  status = vpiImageCreate(image_width_, image_height_, VPI_IMAGE_FORMAT_U8,
                          VPI_BACKEND_CUDA, &vpi_right_rectified_);
  CHECK_VPI_STATUS(status, "Failed to create right rectified image");

  // Disparity output (S16 format, Q11.4 fixed point - scaled by 16)
  // Include VPI_BACKEND_CPU to allow locking for CPU access after GPU processing
  status = vpiImageCreate(image_width_, image_height_, VPI_IMAGE_FORMAT_S16,
                          VPI_BACKEND_CUDA | VPI_BACKEND_CPU, &vpi_disparity_);
  CHECK_VPI_STATUS(status, "Failed to create disparity image");

  // Confidence map (also needs CPU access for potential future use)
  status = vpiImageCreate(image_width_, image_height_, VPI_IMAGE_FORMAT_U16,
                          VPI_BACKEND_CUDA | VPI_BACKEND_CPU, &vpi_confidence_);
  CHECK_VPI_STATUS(status, "Failed to create confidence image");

  // Create filtered disparity buffers if filtering is enabled
  if (enable_disparity_filter_ && (median_filter_size_ > 0 || bilateral_filter_size_ > 0)) {
    status = vpiImageCreate(image_width_, image_height_, VPI_IMAGE_FORMAT_S16,
                            VPI_BACKEND_CUDA | VPI_BACKEND_CPU, &vpi_disparity_filtered_);
    CHECK_VPI_STATUS(status, "Failed to create filtered disparity image");

    // Need temp buffer if we're doing both median and bilateral
    if (median_filter_size_ > 0 && bilateral_filter_size_ > 0) {
      status = vpiImageCreate(image_width_, image_height_, VPI_IMAGE_FORMAT_S16,
                              VPI_BACKEND_CUDA | VPI_BACKEND_CPU, &vpi_disparity_temp_);
      CHECK_VPI_STATUS(status, "Failed to create temp disparity image");
    }
  }

  // Create stereo disparity estimator payload
  VPIStereoDisparityEstimatorCreationParams stereo_params;
  vpiInitStereoDisparityEstimatorCreationParams(&stereo_params);
  stereo_params.maxDisparity = max_disparity_;
  stereo_params.includeDiagonals = 1;

  status = vpiCreateStereoDisparityEstimator(VPI_BACKEND_CUDA, image_width_, image_height_,
                                              VPI_IMAGE_FORMAT_U8, &stereo_params, &stereo_payload_);
  CHECK_VPI_STATUS(status, "Failed to create stereo disparity estimator");

  RCLCPP_INFO(this->get_logger(), "VPI resources created successfully");
  return true;
}

bool StereoDepthEstimator::createWarpMaps()
{
  // Initialize warp grid for output size
  VPIWarpGrid grid;
  memset(&grid, 0, sizeof(grid));
  grid.numHorizRegions = 1;
  grid.numVertRegions = 1;
  grid.horizInterval[0] = 1;  // Dense mapping
  grid.vertInterval[0] = 1;
  grid.regionWidth[0] = image_width_;
  grid.regionHeight[0] = image_height_;

  // Left warp map
  memset(&warp_map_left_, 0, sizeof(warp_map_left_));
  warp_map_left_.grid = grid;
  VPIStatus status = vpiWarpMapAllocData(&warp_map_left_);
  if (status != VPI_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to allocate left warp map");
    return false;
  }

  // Right warp map
  memset(&warp_map_right_, 0, sizeof(warp_map_right_));
  warp_map_right_.grid = grid;
  status = vpiWarpMapAllocData(&warp_map_right_);
  if (status != VPI_SUCCESS) {
    vpiWarpMapFreeData(&warp_map_left_);
    RCLCPP_ERROR(this->get_logger(), "Failed to allocate right warp map");
    return false;
  }

  warp_maps_allocated_ = true;

  // Fill warp maps from OpenCV rectification maps
  // The warp map keypoints store the source coordinates for each destination pixel
  for (int y = 0; y < image_height_; y++) {
    VPIKeypointF32* left_row = reinterpret_cast<VPIKeypointF32*>(
      reinterpret_cast<uint8_t*>(warp_map_left_.keypoints) + y * warp_map_left_.pitchBytes);
    VPIKeypointF32* right_row = reinterpret_cast<VPIKeypointF32*>(
      reinterpret_cast<uint8_t*>(warp_map_right_.keypoints) + y * warp_map_right_.pitchBytes);

    for (int x = 0; x < image_width_; x++) {
      // Get source coordinates from OpenCV maps
      left_row[x].x = map1_left_.at<float>(y, x);
      left_row[x].y = map2_left_.at<float>(y, x);

      right_row[x].x = map1_right_.at<float>(y, x);
      right_row[x].y = map2_right_.at<float>(y, x);
    }
  }

  RCLCPP_INFO(this->get_logger(), "Warp maps created: %dx%d", image_width_, image_height_);
  return true;
}

void StereoDepthEstimator::cleanupVPI()
{
  // Wait for pending operations
  if (vpi_stream_) {
    vpiStreamSync(vpi_stream_);
  }

  // Destroy payloads
  if (stereo_payload_) vpiPayloadDestroy(stereo_payload_);
  if (remap_left_payload_) vpiPayloadDestroy(remap_left_payload_);
  if (remap_right_payload_) vpiPayloadDestroy(remap_right_payload_);

  // Destroy images
  if (vpi_left_input_) vpiImageDestroy(vpi_left_input_);
  if (vpi_right_input_) vpiImageDestroy(vpi_right_input_);
  if (vpi_left_gray_) vpiImageDestroy(vpi_left_gray_);
  if (vpi_right_gray_) vpiImageDestroy(vpi_right_gray_);
  if (vpi_left_rectified_) vpiImageDestroy(vpi_left_rectified_);
  if (vpi_right_rectified_) vpiImageDestroy(vpi_right_rectified_);
  if (vpi_disparity_) vpiImageDestroy(vpi_disparity_);
  if (vpi_disparity_filtered_) vpiImageDestroy(vpi_disparity_filtered_);
  if (vpi_disparity_temp_) vpiImageDestroy(vpi_disparity_temp_);
  if (vpi_confidence_) vpiImageDestroy(vpi_confidence_);

  // Free warp maps
  if (warp_maps_allocated_) {
    vpiWarpMapFreeData(&warp_map_left_);
    vpiWarpMapFreeData(&warp_map_right_);
    warp_maps_allocated_ = false;
  }

  // Destroy stream
  if (vpi_stream_) vpiStreamDestroy(vpi_stream_);

  vpi_stream_ = nullptr;
  stereo_payload_ = nullptr;
  remap_left_payload_ = nullptr;
  remap_right_payload_ = nullptr;
  vpi_left_input_ = nullptr;
  vpi_right_input_ = nullptr;
  vpi_left_gray_ = nullptr;
  vpi_right_gray_ = nullptr;
  vpi_left_rectified_ = nullptr;
  vpi_right_rectified_ = nullptr;
  vpi_disparity_ = nullptr;
  vpi_disparity_filtered_ = nullptr;
  vpi_disparity_temp_ = nullptr;
  vpi_confidence_ = nullptr;
}

void StereoDepthEstimator::stereoImageCallback(sensor_msgs::msg::Image::UniquePtr msg)
{
  if (!vpi_initialized_ || !calibration_loaded_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "VPI or calibration not ready, skipping frame");
    return;
  }

  // Frame rate control - only process 1 out of every N frames
  input_frame_count_++;
  if ((input_frame_count_ % process_every_n_frames_) != 0) {
    return;  // Skip this frame
  }

  // Validate image dimensions
  if (static_cast<int>(msg->width) != stereo_width_ || 
      static_cast<int>(msg->height) != stereo_height_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Image size mismatch: got %dx%d, expected %dx%d",
                         msg->width, msg->height, stereo_width_, stereo_height_);
    return;
  }

  // Wrap ROS message data as cv::Mat (zero-copy)
  cv::Mat stereo_frame;
  if (msg->encoding == "bgr8") {
    stereo_frame = cv::Mat(msg->height, msg->width, CV_8UC3, msg->data.data(), msg->step);
  } else if (msg->encoding == "rgb8") {
    cv::Mat rgb(msg->height, msg->width, CV_8UC3, msg->data.data(), msg->step);
    cv::cvtColor(rgb, stereo_frame, cv::COLOR_RGB2BGR);
  } else if (msg->encoding == "mono8") {
    stereo_frame = cv::Mat(msg->height, msg->width, CV_8UC1, msg->data.data(), msg->step);
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Unsupported encoding: %s", msg->encoding.c_str());
    return;
  }

  // Process the frame
  try {
    processFrame(stereo_frame, rclcpp::Time(msg->header.stamp));
    frame_count_++;

    // Print stats every 100 frames
    if (frame_count_ % 100 == 0) {
      auto now = this->now();
      double elapsed = (now - last_stats_time_).seconds();
      double fps = 100.0 / elapsed;
      RCLCPP_INFO(this->get_logger(), "Depth estimation: %.1f FPS, %d frames processed",
                  fps, frame_count_);
      last_stats_time_ = now;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Error processing frame: %s", e.what());
  }
}

void StereoDepthEstimator::processFrame(const cv::Mat& stereo_frame, const rclcpp::Time& timestamp)
{
  // Split stereo frame into left and right images (zero-copy ROI views)
  // NOTE: After 180° rotation in main_camera_driver:
  //   - First half (x=0 to width) = original RIGHT camera
  //   - Second half (x=width to 2*width) = original LEFT camera
  // The calibrator swaps these when calibrating, so:
  //   - K1/D1/R1/P1 are calibrated for original LEFT (second half)
  //   - K2/D2/R2/P2 are calibrated for original RIGHT (first half)
  // Therefore, we must swap the rectification maps when applying them.
  cv::Mat left_img = stereo_frame(cv::Rect(0, 0, image_width_, image_height_));  // First half = original RIGHT
  cv::Mat right_img = stereo_frame(cv::Rect(image_width_, 0, image_width_, image_height_));  // Second half = original LEFT

  VPIStatus status;

  // ===== HYBRID APPROACH: OpenCV rectification (accurate) + VPI stereo (fast) =====
  
  // Convert to grayscale on CPU
  cv::Mat left_gray, right_gray;
  if (stereo_frame.channels() == 3) {
    cv::cvtColor(left_img, left_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_img, right_gray, cv::COLOR_BGR2GRAY);
  } else {
    left_gray = left_img;
    right_gray = right_img;
  }

  // Rectify using OpenCV
  // Swap maps: left_img (original RIGHT) needs map1_right_, right_img (original LEFT) needs map1_left_
  cv::Mat left_rect, right_rect;
  cv::remap(left_gray, left_rect, map1_right_, map2_right_, cv::INTER_LINEAR);  // Original RIGHT uses K2/D2/R2/P2
  cv::remap(right_gray, right_rect, map1_left_, map2_left_, cv::INTER_LINEAR);   // Original LEFT uses K1/D1/R1/P1

  // Wrap rectified images for VPI (they're already grayscale U8)
  VPIImage vpi_left_wrap = nullptr;
  VPIImage vpi_right_wrap = nullptr;

  status = vpiImageCreateWrapperOpenCVMat(left_rect, VPI_IMAGE_FORMAT_U8,
                                           VPI_BACKEND_CUDA, &vpi_left_wrap);
  if (status != VPI_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to wrap left rectified image");
    return;
  }

  status = vpiImageCreateWrapperOpenCVMat(right_rect, VPI_IMAGE_FORMAT_U8,
                                           VPI_BACKEND_CUDA, &vpi_right_wrap);
  if (status != VPI_SUCCESS) {
    vpiImageDestroy(vpi_left_wrap);
    RCLCPP_ERROR(this->get_logger(), "Failed to wrap right rectified image");
    return;
  }

  // Compute stereo disparity using VPI (GPU-accelerated SGM)
  // Input: OpenCV-rectified images wrapped for VPI
  VPIStereoDisparityEstimatorParams stereo_params;
  vpiInitStereoDisparityEstimatorParams(&stereo_params);
  stereo_params.maxDisparity = max_disparity_;

  status = vpiSubmitStereoDisparityEstimator(vpi_stream_, VPI_BACKEND_CUDA, stereo_payload_,
                                              vpi_left_wrap, vpi_right_wrap,
                                              vpi_disparity_, vpi_confidence_, &stereo_params);
  if (status != VPI_SUCCESS) {
    vpiImageDestroy(vpi_left_wrap);
    vpiImageDestroy(vpi_right_wrap);
    RCLCPP_ERROR(this->get_logger(), "Failed to compute stereo disparity");
    return;
  }

  // Apply disparity filtering (median → bilateral) on GPU
  VPIImage disparity_to_use = vpi_disparity_;  // Points to final disparity to read
  
  if (enable_disparity_filter_) {
    // Stage 1: Median filter (removes outlier speckles)
    if (median_filter_size_ > 0 && vpi_disparity_filtered_) {
      VPIImage median_input = vpi_disparity_;
      VPIImage median_output = (bilateral_filter_size_ > 0 && vpi_disparity_temp_) 
                               ? vpi_disparity_temp_ : vpi_disparity_filtered_;
      
      // MedianFilter signature: stream, backend, input, output, windowWidth, windowHeight, kernel, border
      // Using nullptr kernel for standard median, VPI_BORDER_ZERO for border handling
      status = vpiSubmitMedianFilter(vpi_stream_, VPI_BACKEND_CUDA,
                                     median_input, median_output, 
                                     median_filter_size_, median_filter_size_,  // width, height
                                     nullptr,  // kernel (nullptr = standard median)
                                     VPI_BORDER_ZERO);
      if (status != VPI_SUCCESS) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Median filter failed, using unfiltered disparity");
      } else {
        disparity_to_use = median_output;
      }
    }

    // Stage 2: Bilateral filter (edge-preserving smoothing)
    if (bilateral_filter_size_ > 0 && vpi_disparity_filtered_) {
      VPIImage bilateral_input = (median_filter_size_ > 0 && vpi_disparity_temp_) 
                                  ? vpi_disparity_temp_ : vpi_disparity_;
      VPIImage bilateral_output = vpi_disparity_filtered_;
      
      // BilateralFilter signature: stream, backend, input, output, kernelSize, sigmaSpace, sigmaColor, border
      status = vpiSubmitBilateralFilter(vpi_stream_, VPI_BACKEND_CUDA,
                                        bilateral_input, bilateral_output,
                                        bilateral_filter_size_,
                                        bilateral_sigma_space_,
                                        bilateral_sigma_color_,
                                        VPI_BORDER_ZERO);
      if (status != VPI_SUCCESS) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Bilateral filter failed, using median-only disparity");
      } else {
        disparity_to_use = bilateral_output;
      }
    }
  }

  // Synchronize and get results
  vpiStreamSync(vpi_stream_);

  // Lock disparity image (filtered or raw) and convert to depth
  VPIImageData disparity_data;
  status = vpiImageLockData(disparity_to_use, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &disparity_data);
  if (status != VPI_SUCCESS) {
    char vpi_err[256];
    vpiGetLastStatusMessage(vpi_err, sizeof(vpi_err));
    vpiImageDestroy(vpi_left_wrap);
    vpiImageDestroy(vpi_right_wrap);
    RCLCPP_ERROR(this->get_logger(), "Failed to lock disparity image: %s", vpi_err);
    return;
  }

  // Create depth message with zero-copy publishing
  // Using 16UC1 format (millimeters) - ROS standard for depth images
  auto depth_msg = std::make_unique<sensor_msgs::msg::Image>();
  depth_msg->header.stamp = timestamp;
  depth_msg->header.frame_id = frame_id_;
  depth_msg->height = image_height_;
  depth_msg->width = image_width_;
  depth_msg->encoding = "16UC1";  // Depth in millimeters (uint16)
  depth_msg->is_bigendian = false;
  depth_msg->step = image_width_ * sizeof(uint16_t);
  depth_msg->data.resize(depth_msg->height * depth_msg->step);

  // VPI disparity scaling factor - confirmed from VPI documentation
  // VPI outputs disparity in Q10.5 format (scaled by 32), not Q11.4 (scaled by 16)
  // Reference: VPI example code shows "Disparities are in Q10.5 format, so to map it to float, it gets divided by 32"
  const float DISPARITY_SCALE = 32.0f;  // Convert VPI disparity to pixels (Q10.5 format)

  // Prepare disparity visualization message if needed (do both in single pass)
  std::unique_ptr<sensor_msgs::msg::Image> disp_msg;
  uint8_t* vis_ptr = nullptr;
  float disp_vis_scale = 0.0f;
  
  if (publish_disparity_ && disparity_pub_) {
    disp_msg = std::make_unique<sensor_msgs::msg::Image>();
    disp_msg->header.stamp = timestamp;
    disp_msg->header.frame_id = frame_id_;
    disp_msg->height = image_height_;
    disp_msg->width = image_width_;
    disp_msg->encoding = "mono8";
    disp_msg->is_bigendian = false;
    disp_msg->step = image_width_;
    disp_msg->data.resize(disp_msg->height * disp_msg->step);
    vis_ptr = disp_msg->data.data();
    disp_vis_scale = 255.0f / static_cast<float>(max_disparity_ * DISPARITY_SCALE);  // Scale to pixels
  }

  // Convert disparity to depth using Q matrix (like OpenCV reprojectImageTo3D)
  // VPI outputs disparity in S16 format - need to determine scaling factor
  // The Q matrix expects disparity in pixels (not scaled)
  // Formula: depth = Q[2,3] / (Q[3,2] * disparity + Q[3,3])
  // But we need to convert VPI disparity to pixels first
  uint16_t* depth_ptr = reinterpret_cast<uint16_t*>(depth_msg->data.data());
  const int16_t* disp_ptr = reinterpret_cast<const int16_t*>(disparity_data.buffer.pitch.planes[0].data);
  const int disp_pitch = disparity_data.buffer.pitch.planes[0].pitchBytes / sizeof(int16_t);

  // Extract Q matrix values for depth calculation
  // Q[2,3] = focal length, Q[3,2] = -1/Tx (negative), Q[3,3] = (cx-cx')/Tx
  // Use simpler formula: depth = baseline * focal_length / disparity
  // baseline = |1/Q[3,2]| = |Tx|, focal = Q[2,3]
  const double Q_focal = Q_.at<double>(2, 3);      // Q[2,3] = focal length
  const double Q_baseline_inv = Q_.at<double>(3, 2); // Q[3,2] = -1/Tx (negative)
  const double baseline_m = std::abs(1.0 / Q_baseline_inv); // baseline = |Tx|
  
  // Precompute: baseline * focal_length * scale * 1000 (convert meters to millimeters)
  const float bf_scaled_mm = static_cast<float>(baseline_m * Q_focal * DISPARITY_SCALE * 1000.0);

  // Process row by row for better cache locality
  for (int y = 0; y < image_height_; y++) {
    const int16_t* disp_row = disp_ptr + y * disp_pitch;
    uint16_t* depth_row = depth_ptr + y * image_width_;
    uint8_t* vis_row = vis_ptr ? (vis_ptr + y * image_width_) : nullptr;

    for (int x = 0; x < image_width_; x++) {
      const int16_t disp_raw = disp_row[x];
      
      if (disp_raw > 0) {
        // Use standard formula: depth_mm = baseline_mm * focal_length * scale / disparity_raw
        // This is equivalent to: depth = baseline * focal / (disparity_raw / scale)
        float depth_mm = bf_scaled_mm / static_cast<float>(disp_raw);
        // Clamp to uint16 range (0-65535 mm = 0-65.5m), 0 = invalid
        depth_row[x] = static_cast<uint16_t>(std::clamp(depth_mm, 1.0f, 65535.0f));
      } else {
        depth_row[x] = 0;  // 0 = invalid depth (ROS convention)
      }

      // Disparity visualization in same pass (avoids second lock)
      if (vis_row) {
        vis_row[x] = static_cast<uint8_t>(
          std::clamp(static_cast<float>(disp_raw) * disp_vis_scale, 0.0f, 255.0f));
      }
    }
  }

  vpiImageUnlock(disparity_to_use);

  // Generate and publish point cloud if enabled (before moving depth_msg)
  if (publish_pointcloud_ && pointcloud_pub_) {
    // Get camera intrinsics from P1 (rectified projection matrix)
    const float fx = static_cast<float>(P1_.at<double>(0, 0));
    const float fy = static_cast<float>(P1_.at<double>(1, 1));
    const float cx = static_cast<float>(P1_.at<double>(0, 2));
    const float cy = static_cast<float>(P1_.at<double>(1, 2));

    // Calculate decimated dimensions
    const int pc_width = image_width_ / pointcloud_decimation_;
    const int pc_height = image_height_ / pointcloud_decimation_;
    const int step = pointcloud_decimation_;

    // Create point cloud message
    auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud_msg->header.stamp = timestamp;
    cloud_msg->header.frame_id = frame_id_;
    cloud_msg->height = pc_height;
    cloud_msg->width = pc_width;
    cloud_msg->is_dense = false;
    cloud_msg->is_bigendian = false;

    // Define point cloud fields (x, y, z as float32)
    sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(pc_width * pc_height);

    // Create iterators for x, y, z
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

    // Get pointer to depth data (before we move it)
    const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(depth_msg->data.data());

    // Convert depth to 3D points
    for (int v = 0; v < pc_height; ++v) {
      for (int u = 0; u < pc_width; ++u, ++iter_x, ++iter_y, ++iter_z) {
        // Get depth at decimated position
        int img_u = u * step;
        int img_v = v * step;
        uint16_t depth_mm = depth_data[img_v * image_width_ + img_u];

        if (depth_mm > 0) {
          // Convert to meters
          float z = static_cast<float>(depth_mm) * 0.001f;
          // Back-project to 3D (using pinhole camera model)
          *iter_x = (static_cast<float>(img_u) - cx) * z / fx;
          *iter_y = (static_cast<float>(img_v) - cy) * z / fy;
          *iter_z = z;
        } else {
          // Invalid point
          *iter_x = std::numeric_limits<float>::quiet_NaN();
          *iter_y = std::numeric_limits<float>::quiet_NaN();
          *iter_z = std::numeric_limits<float>::quiet_NaN();
        }
      }
    }

    pointcloud_pub_->publish(std::move(cloud_msg));
  }

  // Publish depth
  depth_pub_->publish(std::move(depth_msg));

  // Publish disparity visualization if enabled
  if (disp_msg) {
    disparity_pub_->publish(std::move(disp_msg));
  }

  // Cleanup temporary wrapped images
  vpiImageDestroy(vpi_left_wrap);
  vpiImageDestroy(vpi_right_wrap);
}

} // namespace maurice_cam

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(maurice_cam::StereoDepthEstimator)

#ifndef BUILDING_COMPONENT_LIBRARY
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<maurice_cam::StereoDepthEstimator>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
#endif

