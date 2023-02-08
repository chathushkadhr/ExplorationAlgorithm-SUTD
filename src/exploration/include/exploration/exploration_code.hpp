#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

#define DEBUG

namespace exploration{


  class MWFCN_Algo : public rclcpp::Node
  {
    public:
      MWFCN_Algo();

    private:
      void timer_callback();
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
      size_t count_;


      std::string nodename;
      std::string map_topic, costmap_topic, trajectory_query_name, output_file, output_map_file;
      std::string robot_frame, robot_base_frame;
      int rateHz;  
      std::string  ns, robot_ano_frame_suffix, robot_ano_frame_preffix;
      int rotation_count, n_robot, this_robot_idx;
      float inflation_radius;    // max 4 degree;
      bool start_condition;
      int no_targets_count;
      float rotation_w[3];
      float rotation_z[3];
      
      

  };
}