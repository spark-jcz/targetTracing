// videoSubscriberH264.cpp —— OK3588 开发板端（ROS2 Foxy）
// 订阅 CompressedImage(format="h264")，后台线程喂给 GStreamer 解码显示；
// 回传时间戳供主机测 RTT；统计帧率/帧间隔/积压/丢帧
#include <fcntl.h>
#include <csignal>

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;
using SteadyClock = std::chrono::steady_clock;
using ImageMsg = sensor_msgs::msg::CompressedImage;

struct NalUnit { size_t offset; size_t length; int type; };
std::vector<NalUnit> splitNals(const std::vector<uint8_t>& d) {
    std::vector<NalUnit> nals;
    const size_t n = d.size();
    size_t i = 0;
    while (i + 3 < n) {
        size_t sc = 0;
        if (d[i]==0 && d[i+1]==0 && d[i+2]==0 && d[i+3]==1) sc = 4;
        else if (d[i]==0 && d[i+1]==0 && d[i+2]==1) sc = 3;
        if (sc == 0) { ++i; continue; }
        const size_t start = i + sc;
        size_t j = start;
        while (j + 3 < n) {
            if ((d[j]==0 && d[j+1]==0 && d[j+2]==1) ||
                (d[j]==0 && d[j+1]==0 && d[j+2]==0 && d[j+3]==1)) break;
            ++j;
        }
        if (j + 3 >= n) j = n;
        nals.push_back({i, j - i, (start < n) ? (d[start] & 0x1F) : -1});
        i = j;
    }
    return nals;
}

class VideoSubscriberH264 : public rclcpp::Node
{
public:
  VideoSubscriberH264()
  : Node("video_subscriber_h264")
  {
    topic_ = declare_parameter<std::string>("topic", "/video/image_compressed");
    ack_topic_ = declare_parameter<std::string>("ack_topic", "/video/frame_ack");
    show_ = declare_parameter<bool>("show", true);
    max_queue_ = declare_parameter<int>("max_queue", 90);
    sink_cmd_ = declare_parameter<std::string>(
      "sink_cmd",
      "gst-launch-1.0 fdsrc fd=0 blocksize=65536 ! "
      "queue max-size-buffers=0 max-size-bytes=4000000 max-size-time=0 ! "
      "h264parse ! mppvideodec ! rkximagesink sync=false");

    sub_ = create_subscription<ImageMsg>(
      topic_, 30, std::bind(&VideoSubscriberH264::onFrame, this, _1));
    ack_pub_ = create_publisher<std_msgs::msg::Header>(ack_topic_, 30);

    if (show_) {
      show_ = startSink();
      if (show_) {
        running_ = true;
        writer_ = std::thread(&VideoSubscriberH264::writerLoop, this);
      }
    }

    t_win_ = SteadyClock::now();
    RCLCPP_INFO(get_logger(),
                "订阅: %s | show=%s | 回传: %s | 显示缓冲上限 %d 帧",
                topic_.c_str(), show_ ? "true" : "false",
                ack_topic_.c_str(), max_queue_);
  }

  ~VideoSubscriberH264() override
  {
    running_ = false;
    cv_.notify_all();
    if (writer_.joinable()) {
      writer_.join();
    }
    if (gst_ != nullptr) {
      pclose(gst_);
      gst_ = nullptr;
    }
  }

private:
  bool startSink()
  {
    gst_ = popen(sink_cmd_.c_str(), "w");
    if (gst_ == nullptr) {
      RCLCPP_WARN(get_logger(), "无法启动显示管道: %s", sink_cmd_.c_str());
      return false;
    }
    const int fd = fileno(gst_);
    if (fd >= 0) {
      // 内核管道缓冲 64KB -> 1MB，吸收运动画面的码流突发
      fcntl(fd, F_SETPIPE_SZ, 1024 * 1024);
    }
    return true;
  }

  // 后台写线程：按 GStreamer 的消费速度喂数据，绝不阻塞订阅回调
  void writerLoop()
  {
    while (rclcpp::ok() && running_) {
      ImageMsg::SharedPtr msg;
      {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait(lk, [this] { return !queue_.empty() || !running_; });
        if (!running_) {
          break;
        }
        if (queue_.empty()) {
          continue;
        }
        msg = queue_.front();
        queue_.pop_front();
      }

      if (gst_ == nullptr || !sink_ok_) {
        continue;
      }
      const size_t n = fwrite(msg->data.data(), 1, msg->data.size(), gst_);
      if (n != msg->data.size()) {
        RCLCPP_WARN(get_logger(), "写入显示管道失败，停止显示");
        sink_ok_ = false;
        continue;
      }
      fflush(gst_);
    }
  }

  void onFrame(const ImageMsg::SharedPtr msg)
  {
    const auto now = SteadyClock::now();
    if (count_ > 0) {
      gap_sum_ += std::chrono::duration<double, std::milli>(now - t_prev_).count();
      ++gap_n_;
    }
    t_prev_ = now;

    // 先回传时间戳（解码前，测纯传输往返）
    std_msgs::msg::Header ack;
    ack.stamp = msg->header.stamp;
    ack.frame_id = msg->header.frame_id;
    ack_pub_->publish(ack);

    if (show_ && !started_) {
      const auto nals = splitNals(msg->data);
      bool has_idr = false;
      for (const auto& nu : nals) {
        if (nu.type == 5) { has_idr = true; break; }
      }
      if (!has_idr) {
        ++dropped_before_idr_;
        return;   // 非关键帧一律丢弃，等 IDR（ack 已回传，不影响 RTT 测量）
      }
      started_ = true;
      RCLCPP_INFO(get_logger(), "已等到首个 IDR，开始送显（起始丢弃 %zu 条非关键帧）",
                  dropped_before_idr_);
    }

    size_t qsize = 0;
    if (show_) {
      bool overflow = false;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        if (static_cast<int>(queue_.size()) >= max_queue_) {
          // 显示端消费过慢：丢最旧帧，避免无界积压
          //（H.264 丢帧会花屏到下一个 IDR，所以要靠显示端跟上来避免）
          const size_t drop = queue_.size() - static_cast<size_t>(max_queue_ / 2);
          for (size_t i = 0; i < drop; ++i) {
            queue_.pop_front();
          }
          dropped_ += drop;
          overflow = true;
        }
        queue_.push_back(msg);
        qsize = queue_.size();
      }
      if (overflow) {
        RCLCPP_WARN(get_logger(),
                    "显示消费过慢，本轮丢弃旧帧，累计丢 %zu 帧", dropped_);
      }
      cv_.notify_one();
    }

    ++count_;
    ++win_count_;

    const double dt =
      std::chrono::duration<double>(SteadyClock::now() - t_win_).count();
    if (dt >= 2.0) {
      RCLCPP_INFO(get_logger(),
                  "累计 %zu 帧 | %.1f fps | 帧间隔 %.1f ms | 显示积压 %zu 帧 | 丢帧 %zu",
                  count_, win_count_ / dt, gap_n_ ? gap_sum_ / gap_n_ : 0.0,
                  qsize, dropped_);
      win_count_ = 0;
      gap_sum_ = 0.0;
      gap_n_ = 0;
      t_win_ = SteadyClock::now();
    }
  }

  std::string topic_;
  std::string ack_topic_;
  std::string sink_cmd_;
  bool show_ = true;
  bool sink_ok_ = true;
  bool running_ = false;
  int max_queue_ = 90;

  FILE * gst_ = nullptr;
  std::thread writer_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::deque<ImageMsg::SharedPtr> queue_;
  size_t dropped_ = 0;

  size_t count_ = 0;
  size_t win_count_ = 0;
  double gap_sum_ = 0.0;
  size_t gap_n_ = 0;
  SteadyClock::time_point t_win_;
  SteadyClock::time_point t_prev_;

  bool started_ = false;              // 是否已等到首个 IDR
  //std::vector<uint8_t> param_sets_;   // 缓存的 SPS/PPS（保险用）
  size_t dropped_before_idr_ = 0;

  rclcpp::Subscription<ImageMsg>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr ack_pub_;
};

int main(int argc, char ** argv)
{
  signal(SIGPIPE, SIG_IGN);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoSubscriberH264>());
  rclcpp::shutdown();
  return 0;
}