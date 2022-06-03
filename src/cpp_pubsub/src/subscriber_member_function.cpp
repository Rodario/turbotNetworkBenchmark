#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "bench_pack/msg/num.hpp"
#include "bench_pack/msg/bench.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<bench_pack::msg::Bench>(
      "sync", 12, std::bind(&MinimalSubscriber::topic_callback, this, _1));

      publisher_ = this->create_publisher<bench_pack::msg::Bench>("ack", 12);
    }

  private:
    void topic_callback(const bench_pack::msg::Bench & msg) const
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "I heard: ID '" << msg.id << " | " << msg.start_time << "'");
      auto message = bench_pack::msg::Bench();
      message.id =  1;
      message.start_time = msg.start_time;
      publisher_->publish(message);
    }
    // sub
    rclcpp::Subscription<bench_pack::msg::Bench>::SharedPtr subscription_;

    // pub
    rclcpp::Publisher<bench_pack::msg::Bench>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}