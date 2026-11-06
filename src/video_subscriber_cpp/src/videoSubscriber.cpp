// videoSubscriber.cpp —— OK3588 开发板端（ROS2 Foxy）
// 订阅 CompressedImage，立即回传时间戳供主机测 RTT；可选解码显示
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/header.hpp"
#include <opencv2/opencv.hpp>

using std::placeholders::_1;
using SteadyClock = std::chrono::steady_clock;

class VideoSubscriber : public rclcpp::Node
{
public:
  VideoSubscriber()
  : Node("video_subscriber_cpp"),
    count_(0), win_count_(0), gap_sum_(0.0), gap_n_(0)
  {
    topic_ = declare_parameter<std::string>("topic", "/video/image_compressed");
    show_ = declare_parameter<bool>("show", true);

    sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      topic_, 10, std::bind(&VideoSubscriber::onFrame, this, _1));
    ack_pub_ = create_publisher<std_msgs::msg::Header>("/video/frame_ack", 10);

    t_win_ = SteadyClock::now();
    RCLCPP_INFO(get_logger(), "订阅: %s | show=%s | 回传: /video/frame_ack",
                topic_.c_str(), show_ ? "true" : "false");
  }

private:
  void onFrame(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    const auto now = SteadyClock::now();
    if (count_ > 0) {
      gap_sum_ += std::chrono::duration<double, std::milli>(now - t_prev_).count();
      ++gap_n_;
    }
    t_prev_ = now;

    // 先回传时间戳（放在解码之前，测的是纯传输往返）
    std_msgs::msg::Header ack;
    ack.stamp = msg->header.stamp;
    ack.frame_id = msg->header.frame_id;
    ack_pub_->publish(ack);

    // 只有需要显示时才解码
    if (show_) {
      cv::Mat raw(1, static_cast<int>(msg->data.size()),
                  CV_8UC1, const_cast<uint8_t *>(msg->data.data()));
      cv::Mat frame = cv::imdecode(raw, cv::IMREAD_COLOR);
      if (!frame.empty()) {
        try {
          cv::imshow("video from WSL", frame);
          cv::waitKey(1);
        } catch (const cv::Exception &) {
          RCLCPP_WARN(get_logger(), "显示不可用（OpenCV 无 GUI 后端），转无头模式");
          show_ = false;
        }
      }
    }

    ++count_;
    ++win_count_;

    const double dt =
      std::chrono::duration<double>(SteadyClock::now() - t_win_).count();
    if (dt >= 2.0) {
      RCLCPP_INFO(get_logger(), "累计 %zu 帧 | %.1f fps | 帧间隔 %.1f ms",
                  count_, win_count_ / dt, gap_n_ ? gap_sum_ / gap_n_ : 0.0);
      win_count_ = 0;
      gap_sum_ = 0.0;
      gap_n_ = 0;
      t_win_ = SteadyClock::now();
    }
  }

  std::string topic_;
  bool show_;
  size_t count_;
  size_t win_count_;
  double gap_sum_;
  size_t gap_n_;
  SteadyClock::time_point t_win_;
  SteadyClock::time_point t_prev_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr ack_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoSubscriber>());
  try {
    cv::destroyAllWindows();      // 无 GUI 后端时会抛异常，吞掉
  } catch (const cv::Exception &) {
  }
  rclcpp::shutdown();
  return 0;
}