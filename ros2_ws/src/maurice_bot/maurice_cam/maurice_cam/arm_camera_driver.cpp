#include "maurice_cam/arm_camera_driver.hpp"
#include <filesystem>

using namespace std::chrono_literals;

namespace maurice_cam
{

ArmCameraDriver::ArmCameraDriver(const rclcpp::NodeOptions & options) 
  : Node("arm_camera_driver", options), 
    invalid_buffer_index_count_(0) {
    RCLCPP_DEBUG(this->get_logger(), "Initializing arm camera driver...");
    RCLCPP_DEBUG(this->get_logger(), "OpenCV version: %s", CV_VERSION);

    // Parameters
    this->declare_parameter<std::string>("camera_symlink", "Arducam");
    this->declare_parameter<int>("width", 640);
    this->declare_parameter<int>("height", 480);
    this->declare_parameter<double>("fps", 30.0);
    this->declare_parameter<std::string>("pixel_format", "YUYV");
    this->declare_parameter<bool>("publish_compressed", false);
    this->declare_parameter<int>("compressed_frame_interval", 5);

    // Get parameters
    std::string camera_symlink = this->get_parameter("camera_symlink").as_string();
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    fps_ = static_cast<int>(this->get_parameter("fps").as_double());
    publish_compressed_ = this->get_parameter("publish_compressed").as_bool();
    compressed_frame_interval_ = this->get_parameter("compressed_frame_interval").as_int();

    // Find camera symlink by pattern matching
    std::string camera_pattern = camera_symlink;  // Parameter now contains pattern
    std::string symlink_path;
    std::string v4l_dir = "/dev/v4l/by-id/";
    
    bool found = false;
    if (std::filesystem::exists(v4l_dir)) {
      for (const auto& entry : std::filesystem::directory_iterator(v4l_dir)) {
        std::string filename = entry.path().filename().string();
        if (filename.find(camera_pattern) != std::string::npos) {
          symlink_path = entry.path().string();
          found = true;
          RCLCPP_INFO(this->get_logger(), "Found camera symlink matching pattern '%s': %s", 
                      camera_pattern.c_str(), filename.c_str());
          break;
        }
      }
    }
    
    if (!found) {
      RCLCPP_ERROR(this->get_logger(), "Camera symlink matching pattern '%s' not found in %s", 
                   camera_pattern.c_str(), v4l_dir.c_str());
      throw std::runtime_error("Camera symlink not found");
    }
    
    // Resolve the symlink to get actual device path
    std::string resolved_path = std::filesystem::read_symlink(symlink_path).string();
    
    // Handle relative paths properly
    if (resolved_path.find("/dev/") == 0) {
        device_path_ = resolved_path;
    } else {
        std::filesystem::path symlink_dir = std::filesystem::path(symlink_path).parent_path();
        std::filesystem::path full_path = std::filesystem::canonical(symlink_dir / resolved_path);
        device_path_ = full_path.string();
    }

    RCLCPP_INFO(this->get_logger(), "=== Maurice Arm Camera Driver ===");
    RCLCPP_INFO(this->get_logger(), "Camera pattern: %s", camera_pattern.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera symlink: %s", symlink_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Resolved device: %s", device_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d", width_, height_);
    RCLCPP_INFO(this->get_logger(), "FPS: %d", fps_);
    RCLCPP_INFO(this->get_logger(), "Pixel Format: YUYV");

    // Create publishers with sensor data QoS profile
    rclcpp::QoS qos = rclcpp::SensorDataQoS()
        .keep_last(10)  // Keep last 10 messages
        .best_effort()  // Best effort delivery for real-time data
        .durability_volatile();  // Volatile durability

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/mars/arm/image_raw", qos);
    
    if (publish_compressed_) {
        compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/mars/arm/image_raw/compressed", qos);
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Created publishers with sensor data QoS:");
    RCLCPP_DEBUG(this->get_logger(), "  Raw image: /mars/arm/image_raw");
    if (publish_compressed_) {
        RCLCPP_DEBUG(this->get_logger(), "  Compressed image: /mars/arm/image_raw/compressed");
    }
    RCLCPP_DEBUG(this->get_logger(), "  QoS Settings:");
    RCLCPP_DEBUG(this->get_logger(), "    - Reliability: BEST_EFFORT");
    RCLCPP_DEBUG(this->get_logger(), "    - Durability: VOLATILE");
    RCLCPP_DEBUG(this->get_logger(), "    - History: KEEP_LAST (10)");
    RCLCPP_DEBUG(this->get_logger(), "    - Deadline: Default");
    RCLCPP_DEBUG(this->get_logger(), "    - Liveliness: Default");

    // Initialize camera
    if (!init_camera()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize arm camera");
        return;
    }

    // Create timer for frame capture
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / fps_),
        std::bind(&ArmCameraDriver::capture_and_publish, this));
    
    RCLCPP_INFO(this->get_logger(), "Arm camera driver ready");
}

ArmCameraDriver::~ArmCameraDriver() {
    if (fd_ >= 0) {
        RCLCPP_INFO(this->get_logger(), "Closing arm camera device");
        close(fd_);
    }
}

bool ArmCameraDriver::init_camera() {
    RCLCPP_DEBUG(this->get_logger(), "Opening camera device: %s", device_path_.c_str());
    
    // Check if device file exists first
    if (!std::filesystem::exists(device_path_)) {
        RCLCPP_ERROR(this->get_logger(), "Device file does not exist: %s", device_path_.c_str());
        return false;
    }

    fd_ = open(device_path_.c_str(), O_RDWR | O_NONBLOCK);  // Add O_NONBLOCK
    if (fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera device: %s", strerror(errno));
        return false;
    }

    struct v4l2_capability cap;
    if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to query device capabilities");
        return false;
    }

    RCLCPP_DEBUG(this->get_logger(), "Camera capabilities:");
    RCLCPP_DEBUG(this->get_logger(), "  Driver: %s", cap.driver);
    RCLCPP_DEBUG(this->get_logger(), "  Card: %s", cap.card);
    RCLCPP_DEBUG(this->get_logger(), "  Bus info: %s", cap.bus_info);

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        RCLCPP_ERROR(this->get_logger(), "Device does not support video capture");
        return false;
    }

    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width_;
    fmt.fmt.pix.height = height_;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set format");
        return false;
    }

    RCLCPP_DEBUG(this->get_logger(), "Set camera format:");
    RCLCPP_DEBUG(this->get_logger(), "  Width: %d", fmt.fmt.pix.width);
    RCLCPP_DEBUG(this->get_logger(), "  Height: %d", fmt.fmt.pix.height);
    RCLCPP_DEBUG(this->get_logger(), "  Pixel Format: YUYV");

    struct v4l2_streamparm streamparm = {};
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    streamparm.parm.capture.timeperframe.numerator = 1;
    streamparm.parm.capture.timeperframe.denominator = fps_;

    if (ioctl(fd_, VIDIOC_S_PARM, &streamparm) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set frame rate");
        return false;
    }

    RCLCPP_DEBUG(this->get_logger(), "Set frame rate: %d fps", fps_);

    // Request buffers
    struct v4l2_requestbuffers req = {};
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to request buffers");
        return false;
    }

    RCLCPP_DEBUG(this->get_logger(), "Requested %d buffers", req.count);

    // Map buffers
    for (unsigned int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to query buffer %d", i);
            return false;
        }

        buffers_[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
        if (buffers_[i].start == MAP_FAILED) {
            RCLCPP_ERROR(this->get_logger(), "Failed to map buffer %d", i);
            return false;
        }
        buffers_[i].length = buf.length;
        RCLCPP_DEBUG(this->get_logger(), "Mapped buffer %d: %u bytes", i, static_cast<unsigned int>(buf.length));
    }

    // Queue buffers
    for (unsigned int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to queue buffer %d", i);
            return false;
        }
    }

    // Start streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start streaming");
        return false;
    }

    RCLCPP_DEBUG(this->get_logger(), "Camera streaming started");
    return true;
}

void ArmCameraDriver::handle_stream_error() {
    RCLCPP_ERROR(this->get_logger(), "Stream error occurred (errno=%d: %s), attempting to recover...", errno, strerror(errno));
    
    // Unmap buffers first
    for (int i = 0; i < 4; i++) {
        if (buffers_[i].start && buffers_[i].start != MAP_FAILED) {
            munmap(buffers_[i].start, buffers_[i].length);
            buffers_[i].start = nullptr;
            buffers_[i].length = 0;
        }
    }
    
    // Close current connection
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
    
    // Wait a bit for device to stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // Try to reinitialize camera
    if (init_camera()) {
        RCLCPP_INFO(this->get_logger(), "Successfully reconnected to camera");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to reconnect to camera");
    }
}

void ArmCameraDriver::capture_and_publish() {
    static int frame_count = 0;
    frame_count++;
    
    // Log every 1000 frames for health monitoring (~33 seconds at 30 fps)
    if (frame_count % 1000 == 0) {
        RCLCPP_INFO(this->get_logger(), "Camera health check - Frame %d, Device: %s", 
                    frame_count, device_path_.c_str());
    }
    
    // Check if camera is still connected before attempting capture
    if (fd_ < 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Camera not connected, attempting to reconnect...");
        
        if (init_camera()) {
            RCLCPP_INFO(this->get_logger(), "Camera reconnected successfully");
        }
        return;
    }

    // Check if current device file still exists
    if (!std::filesystem::exists(device_path_)) {
        RCLCPP_ERROR(this->get_logger(), "Current device file disappeared: %s, attempting to recover...", device_path_.c_str());
        handle_stream_error();
        return;
    }

    // Use select to check if data is available (with timeout)
    fd_set fds;
    struct timeval tv;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms timeout

    int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "select() failed: %s", strerror(errno));
        handle_stream_error();
        return;
    } else if (ret == 0) {
        // Timeout - no data available
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No data available from camera (timeout)");
        return;
    }

    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "VIDIOC_DQBUF failed (idx=%u, errno=%d): %s", buf.index, errno, strerror(errno));
        handle_stream_error();
        return;
    }

    // Validate buffer index with retry logic
    if (buf.index >= 4) {
        invalid_buffer_index_count_++;
        RCLCPP_ERROR(this->get_logger(), 
                     "[FRAME %d] Invalid buffer index: %u (count: %d/%d) - Device: %s", 
                     frame_count, buf.index, invalid_buffer_index_count_, MAX_ERROR_COUNT, device_path_.c_str());
        
        if (invalid_buffer_index_count_ >= MAX_ERROR_COUNT) {
            RCLCPP_ERROR(this->get_logger(), "Too many invalid buffer index errors, attempting recovery");
            invalid_buffer_index_count_ = 0;  // Reset counter
            handle_stream_error();
        }
        return;
    } else {
        invalid_buffer_index_count_ = 0;  // Reset counter on success
    }

    // Check if buffer data looks valid (keep original behavior)
    if (!buffers_[buf.index].start || buf.bytesused == 0) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Invalid buffer data - idx:%u, bytesused:%u, buffer_size:%zu, device:%s, frame:%d", 
                     buf.index, buf.bytesused, buffers_[buf.index].length, device_path_.c_str(), frame_count);
        // Still try to requeue the buffer
        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to requeue invalid buffer");
            handle_stream_error();
        }
        return;
    }

    try {
        // Convert YUYV to BGR
        cv::Mat yuyv(height_, width_, CV_8UC2, buffers_[buf.index].start);
        cv::Mat bgr;
        cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);

        // Validate converted image (keep original behavior)
        if (bgr.empty() || bgr.cols != width_ || bgr.rows != height_) {
            RCLCPP_ERROR(this->get_logger(), "Invalid converted image (idx=%u)", buf.index);
            // Don't return here, still need to requeue buffer
        } else {
            auto stamp = this->now();
            
            // Create raw image message as UniquePtr for zero-copy intra-process
            auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
            img_msg->header.stamp = stamp;
            img_msg->header.frame_id = "arm_camera";
            img_msg->height = bgr.rows;
            img_msg->width = bgr.cols;
            img_msg->encoding = "bgr8";
            img_msg->is_bigendian = false;
            img_msg->step = bgr.cols * 3;
            img_msg->data.assign(bgr.data, bgr.data + (bgr.rows * img_msg->step));

            // Publish with std::move() for zero-copy intra-process communication
            // Inter-process subscribers (via DDS) still receive serialized copies automatically
            image_pub_->publish(std::move(img_msg));

            // Conditionally create and publish compressed image at specified interval
            if (publish_compressed_) {
                compressed_frame_counter_++;
                if (compressed_frame_counter_ >= compressed_frame_interval_) {
                    compressed_frame_counter_ = 0;
                    
                    auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
                    compressed_msg->header.stamp = stamp;
                    compressed_msg->header.frame_id = "arm_camera";
                    compressed_msg->format = "jpeg";

                    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
                    if (!cv::imencode(".jpg", bgr, compressed_msg->data, params)) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to compress image to JPEG");
                    } else {
                        compressed_pub_->publish(std::move(compressed_msg));
                    }
                }
            }

            // Debug level for successful operations
            RCLCPP_DEBUG(this->get_logger(), "Successfully processed frame %d, buffer idx %u", frame_count, buf.index);
        }
    } catch (const cv::Exception &e) {
        RCLCPP_ERROR(this->get_logger(),
                     "OpenCV exception on buffer %u: %s", buf.index, e.what());
        handle_stream_error();
        return;
    }

    // Requeue the buffer
    if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "VIDIOC_QBUF failed (idx=%u, errno=%d): %s", buf.index, errno, strerror(errno));
        handle_stream_error();
    }
}

} // namespace maurice_cam

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(maurice_cam::ArmCameraDriver)

#ifndef BUILDING_COMPONENT_LIBRARY
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<maurice_cam::ArmCameraDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#endif

