#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "bench_pack/msg/num.hpp"
#include "bench_pack/msg/bench.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
      public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<bench_pack::msg::Bench>("sync", 12);
      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));

      subscription_ = this->create_subscription<bench_pack::msg::Bench>(
        "ack", 12, std::bind(&MinimalPublisher::ack_callback, this, _1)
      );
    }

  private:
    void timer_callback()
    {
      auto message = bench_pack::msg::Bench();
      auto now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

      message.id = 0;
      message.start_time = now;
      //RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.num << "'");
      publisher_->publish(message);
    }

    void ack_callback(const bench_pack::msg::Bench &msg) const
    {
      // take current time in milliseconds since epoch
      auto now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
      RCLCPP_INFO_STREAM(this->get_logger(), "["<< (uint8_t) msg.id << "] Duration: {" << now - msg.start_time << "}");
    }

    // publisher declarations
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<bench_pack::msg::Bench>::SharedPtr publisher_;
    size_t count_;

    // subscriber declarations
    rclcpp::Subscription<bench_pack::msg::Bench>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}