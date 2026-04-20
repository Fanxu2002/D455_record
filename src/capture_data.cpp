#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

class RealSenseDataExporter : public rclcpp::Node
{
public:
  RealSenseDataExporter()
      : Node("realsense_data_exporter")
  {
    output_dir_ = this->declare_parameter<std::string>("output_dir", "scene_export");
    rgb_topic_ = this->declare_parameter<std::string>("rgb_topic", "/camera/camera/color/image_raw");
    depth_topic_ = this->declare_parameter<std::string>("depth_topic", "/camera/camera/depth/image_rect_raw");
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/camera/camera/imu");
    color_info_topic_ = this->declare_parameter<std::string>("color_info_topic", "/camera/camera/color/camera_info");
    depth_info_topic_ = this->declare_parameter<std::string>("depth_info_topic", "/camera/camera/depth/camera_info");

    save_rgb_video_ = this->declare_parameter<bool>("save_rgb_video", true);
    save_depth_ = this->declare_parameter<bool>("save_depth", true);
    save_imu_ = this->declare_parameter<bool>("save_imu", true);

    rgb_video_fps_ = this->declare_parameter<double>("rgb_video_fps", 30.0);
    save_every_n_depth_frames_ = this->declare_parameter<int>("save_every_n_depth_frames", 6);
    rgb_video_filename_ = this->declare_parameter<std::string>("rgb_video_filename", "rgb_video.avi");

    depth_dir_ = fs::path(output_dir_) / "depth_raw";
    imu_dir_ = fs::path(output_dir_) / "imu";

    fs::create_directories(output_dir_);
    fs::create_directories(depth_dir_);
    fs::create_directories(imu_dir_);

    timestamps_file_.open((fs::path(output_dir_) / "timestamps.csv").string(), std::ios::out);
    timestamps_file_ << "index,type,stamp_sec,stamp_nanosec,filename\n";

    imu_file_.open((imu_dir_ / "imu.csv").string(), std::ios::out);
    imu_file_ << "stamp_sec,stamp_nanosec,"
              << "orientation_x,orientation_y,orientation_z,orientation_w,"
              << "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
              << "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z\n";

    using std::placeholders::_1;

    rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        rgb_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RealSenseDataExporter::rgbCallback, this, _1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RealSenseDataExporter::depthCallback, this, _1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RealSenseDataExporter::imuCallback, this, _1));

    color_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        color_info_topic_, 10,
        std::bind(&RealSenseDataExporter::colorInfoCallback, this, _1));

    depth_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        depth_info_topic_, 10,
        std::bind(&RealSenseDataExporter::depthInfoCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Exporting data to: %s", fs::absolute(output_dir_).c_str());
  }

  ~RealSenseDataExporter() override
  {
    if (rgb_writer_.isOpened())
    {
      rgb_writer_.release();
    }
    if (timestamps_file_.is_open())
    {
      timestamps_file_.close();
    }
    if (imu_file_.is_open())
    {
      imu_file_.close();
    }
  }

private:
  std::string makeStampedFilename(const builtin_interfaces::msg::Time &stamp, const std::string &ext)
  {
    std::ostringstream oss;
    oss << stamp.sec << "_" << std::setw(9) << std::setfill('0') << stamp.nanosec << ext;
    return oss.str();
  }

  void writeCameraInfoJson(
      const sensor_msgs::msg::CameraInfo &msg,
      const std::string &name)
  {
    fs::path path = fs::path(output_dir_) / (name + "_camera_info.json");
    std::ofstream ofs(path.string(), std::ios::out);
    if (!ofs.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s", path.c_str());
      return;
    }

    ofs << "{\n";
    ofs << "  \"width\": " << msg.width << ",\n";
    ofs << "  \"height\": " << msg.height << ",\n";
    ofs << "  \"distortion_model\": \"" << msg.distortion_model << "\",\n";

    ofs << "  \"d\": [";
    for (size_t i = 0; i < msg.d.size(); ++i)
    {
      ofs << msg.d[i];
      if (i + 1 < msg.d.size())
        ofs << ", ";
    }
    ofs << "],\n";

    ofs << "  \"k\": [";
    for (size_t i = 0; i < msg.k.size(); ++i)
    {
      ofs << msg.k[i];
      if (i + 1 < msg.k.size())
        ofs << ", ";
    }
    ofs << "],\n";

    ofs << "  \"r\": [";
    for (size_t i = 0; i < msg.r.size(); ++i)
    {
      ofs << msg.r[i];
      if (i + 1 < msg.r.size())
        ofs << ", ";
    }
    ofs << "],\n";

    ofs << "  \"p\": [";
    for (size_t i = 0; i < msg.p.size(); ++i)
    {
      ofs << msg.p[i];
      if (i + 1 < msg.p.size())
        ofs << ", ";
    }
    ofs << "]\n";

    ofs << "}\n";
    ofs.close();
  }

  void colorInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (color_info_saved_)
      return;
    writeCameraInfoJson(*msg, "color");
    color_info_saved_ = true;
    RCLCPP_INFO(this->get_logger(), "Saved color camera info.");
  }

  void depthInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (depth_info_saved_)
      return;
    writeCameraInfoJson(*msg, "depth");
    depth_info_saved_ = true;
    RCLCPP_INFO(this->get_logger(), "Saved depth camera info.");
  }

  void openRgbWriterIfNeeded(const cv::Mat &image)
  {
    if (rgb_writer_.isOpened())
      return;

    fs::path video_path = fs::path(output_dir_) / rgb_video_filename_;

    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    bool ok = rgb_writer_.open(
        video_path.string(),
        fourcc,
        rgb_video_fps_,
        image.size(),
        image.channels() == 3);

    if (!ok)
    {
      throw std::runtime_error("Failed to open RGB video writer: " + video_path.string());
    }

    RCLCPP_INFO(this->get_logger(), "RGB video writer opened: %s", video_path.c_str());
  }

  void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!save_rgb_video_)
      return;

    try
    {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
      cv::Mat image = cv_ptr->image;

      if (image.empty())
      {
        RCLCPP_ERROR(this->get_logger(), "RGB image is empty.");
        return;
      }

      if (msg->encoding == "rgb8")
      {
        cv::Mat bgr_image;
        cv::cvtColor(image, bgr_image, cv::COLOR_RGB2BGR);
        image = bgr_image;
      }
      else if (msg->encoding == "rgba8")
      {
        cv::Mat bgr_image;
        cv::cvtColor(image, bgr_image, cv::COLOR_RGBA2BGR);
        image = bgr_image;
      }
      else if (msg->encoding == "bgra8")
      {
        cv::Mat bgr_image;
        cv::cvtColor(image, bgr_image, cv::COLOR_BGRA2BGR);
        image = bgr_image;
      }
      else if (msg->encoding == "mono8")
      {
        cv::Mat bgr_image;
        cv::cvtColor(image, bgr_image, cv::COLOR_GRAY2BGR);
        image = bgr_image;
      }
      else if (msg->encoding != "bgr8")
      {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Unexpected RGB encoding: %s", msg->encoding.c_str());
      }

      openRgbWriterIfNeeded(image);
      rgb_writer_.write(image);

      {
        std::lock_guard<std::mutex> lock(file_mutex_);
        timestamps_file_ << rgb_count_++ << ",rgb_video_frame,"
                         << msg->header.stamp.sec << ","
                         << msg->header.stamp.nanosec << ","
                         << rgb_video_filename_ << "\n";
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(
          this->get_logger(),
          "RGB video save failed. encoding=%s width=%u height=%u step=%u error=%s",
          msg->encoding.c_str(), msg->width, msg->height, msg->step, e.what());
    }
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!save_depth_)
      return;

    depth_frame_counter_++;
    if (save_every_n_depth_frames_ > 1 &&
        (depth_frame_counter_ % save_every_n_depth_frames_) != 0)
    {
      return;
    }

    try
    {
      cv::Mat image = cv_bridge::toCvCopy(msg, msg->encoding)->image;

      if (image.type() != CV_16UC1)
      {
        cv::Mat converted;
        image.convertTo(converted, CV_16UC1);
        image = converted;
      }

      std::string filename = makeStampedFilename(msg->header.stamp, ".png");
      fs::path out_path = depth_dir_ / filename;

      cv::imwrite(out_path.string(), image, {cv::IMWRITE_PNG_COMPRESSION, 0});

      {
        std::lock_guard<std::mutex> lock(file_mutex_);
        timestamps_file_ << depth_count_++ << ",depth,"
                         << msg->header.stamp.sec << ","
                         << msg->header.stamp.nanosec << ","
                         << filename << "\n";
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Depth save failed: %s", e.what());
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!save_imu_)
      return;

    std::lock_guard<std::mutex> lock(file_mutex_);
    imu_file_
        << msg->header.stamp.sec << ","
        << msg->header.stamp.nanosec << ","
        << msg->orientation.x << ","
        << msg->orientation.y << ","
        << msg->orientation.z << ","
        << msg->orientation.w << ","
        << msg->angular_velocity.x << ","
        << msg->angular_velocity.y << ","
        << msg->angular_velocity.z << ","
        << msg->linear_acceleration.x << ","
        << msg->linear_acceleration.y << ","
        << msg->linear_acceleration.z << "\n";
  }

private:
  std::string output_dir_;
  std::string rgb_topic_;
  std::string depth_topic_;
  std::string imu_topic_;
  std::string color_info_topic_;
  std::string depth_info_topic_;
  std::string rgb_video_filename_;

  bool save_rgb_video_{true};
  bool save_depth_{true};
  bool save_imu_{true};

  double rgb_video_fps_{30.0};
  int save_every_n_depth_frames_{6};

  fs::path depth_dir_;
  fs::path imu_dir_;

  std::ofstream timestamps_file_;
  std::ofstream imu_file_;
  std::mutex file_mutex_;

  cv::VideoWriter rgb_writer_;

  size_t rgb_count_{0};
  size_t depth_count_{0};
  size_t depth_frame_counter_{0};

  bool color_info_saved_{false};
  bool depth_info_saved_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealSenseDataExporter>());
  rclcpp::shutdown();
  return 0;
}
