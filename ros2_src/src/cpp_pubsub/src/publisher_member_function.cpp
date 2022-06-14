#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

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
      std::srand(std::time(nullptr)); // use current time as seed for random generator
      int random_variable = std::rand()%10000;
      file_name = file_name + "_" + std::to_string(random_variable) + ".csv";
      MinimalPublisher::write_line_to_csv( file_name, "ID,RT_DURATION,");
    }

  private:
    void timer_callback()
    {
      auto message = bench_pack::msg::Bench();
      auto now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
      int8_t id = 0;
      message.id = id;
      message.start_time = now;
      //RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.num << "'");
      publisher_->publish(message);
    }

    void ack_callback(const bench_pack::msg::Bench &msg) const
    {
      // take current time in milliseconds since epoch
      auto now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
      RCLCPP_INFO_STREAM(this->get_logger(), "["<< (int) msg.id << "] Duration: {" << now - msg.start_time << "}");
      // write to file
      std::ostringstream line;
      line << (int) msg.id << "," << now - msg.start_time << ",";
      MinimalPublisher::write_line_to_csv(file_name, line.str());
    }

    void write_line_to_csv (const std::string file_name, const std::string line) const {      
      std::ofstream myfile;
      // creates file if not available and appends if available
      myfile.open(file_name, std::fstream::in | std::fstream::out | std::fstream::app);
      myfile << line << "\n";
      myfile.close();
    }

    // publisher declarations
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<bench_pack::msg::Bench>::SharedPtr publisher_;
    size_t count_;

    // subscriber declarations
    rclcpp::Subscription<bench_pack::msg::Bench>::SharedPtr subscription_;

    std::string file_name = "ros2_roundtrip_log";
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}