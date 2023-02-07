#include <exploration/exploration_code.hpp>

using namespace exploration;

SimplePublisher:: SimplePublisher() : Node("simple_publisher")
{
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&SimplePublisher::timer_callback, this));
}


void SimplePublisher:: timer_callback()
{
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
}

rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    
