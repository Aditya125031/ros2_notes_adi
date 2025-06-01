#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class RandomNumberSubscriber : public rclcpp::Node
{
public:
  RandomNumberSubscriber()
  : Node("number_subscriber")
  {
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
 	    auto qos_profile = rclcpp::QoS(10).reliable();
	    auto qos_profile = rclcpp::QoS(10).best_effort();

    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "number", 10,
      std::bind(&RandomNumberSubscriber::topic_callback, this, std::placeholders::_1)
    );
  }

private:
  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    int received = msg->data;
    int squared = received * received;
    RCLCPP_INFO(this->get_logger(), "Received: %d | Squared: %d", received, squared);
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RandomNumberSubscriber>());
  rclcpp::shutdown();
  return 0;
}
