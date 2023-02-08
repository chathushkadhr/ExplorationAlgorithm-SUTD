#include <exploration/exploration_code.hpp>

using namespace exploration;


MWFCN_Algo:: MWFCN_Algo() : 
      Node("MWFCN_node"), 
      rotation_count(0),
      start_condition(true),
      no_targets_count(0),
      rotation_w({0.866,  0.500, 1.0}),
      rotation_z({0.5  , -0.866, 0.0})
{     
      nodename=MWFCN_Algo::get_name();

      this->declare_parameter("map_topic", "robot1/map");
      std::string map_topic=this->get_parameter("map_topic").get_parameter_value().get<std::string>();


      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MWFCN_Algo::timer_callback, this));

      #ifdef DEBUG
      RCLCPP_INFO_STREAM(this->get_logger(), "Map topic: " << map_topic);
      
      #endif
}


void MWFCN_Algo:: timer_callback()
{
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      std::string x = std::to_string(rotation_w[0]);
      // RCLCPP_INFO(this->get_logger(), map_topic.c_str());
      publisher_->publish(message);
}


 

    
