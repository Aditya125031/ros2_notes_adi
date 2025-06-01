#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class randomnumberpublisher : public rclcpp::Node
{
    public:
        randomnumberpublisher()
        :Node("number_publisher")
        {
    	    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
 	    auto qos_profile = rclcpp::QoS(10).reliable();
	    auto qos_profile = rclcpp::QoS(10).best_effort();
	    
            publisher_=this->create_publisher<std_msgs::msg::Int32>("number",10);
            timer_=this->create_wall_timer
            (
                std::chrono::seconds(1),
                std::bind(&randomnumberpublisher::publish_number, this)
            );
        }
    private:
        void publish_number()
        {
            static std::random_device rd;
            static std::mt19937 gen(rd());
            static std::uniform_int_distribution<> dis(1, 100);

            int random_number=dis(gen);
            auto msg=std_msgs::msg::Int32();
            msg.data=random_number;

            RCLCPP_INFO(this->get_logger(),"Publishing: %d",msg.data);
            publisher_->publish(msg);

        }
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<randomnumberpublisher>());
  rclcpp::shutdown();
  return 0;
}
