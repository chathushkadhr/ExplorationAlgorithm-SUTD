#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
using namespace std::chrono_literals;

#define DEBUG

namespace exploration{


  class MWFCN_Algo : public rclcpp::Node
  {
    public:
      MWFCN_Algo();
      nav_msgs::msg::OccupancyGrid mapData,costmapData;
      geometry_msgs::msg::PointStamped clickedpoint;
      visualization_msgs::msg::Marker points,line;
      geometry_msgs::msg::Point p;  

    private:
      void timer_callback();
      void debug_param();
      void mapCallBack(const nav_msgs::msg::OccupancyGrid::ConstPtr & msg);
      void costmapMergedCallBack(const nav_msgs::msg::OccupancyGrid::ConstPtr & msg);
      void rvizCallBack(const geometry_msgs::msg::PointStamped::ConstPtr & msg);

      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
      size_t count_;


      std::string map_topic,costmap_topic,trajectory_query_name, output_file, output_map_file;
      std::string robot_frame, robot_base_frame;
      int rateHz;  
      std::string  ns, robot_ano_frame_suffix, robot_ano_frame_preffix;
      int rotation_count, n_robot, this_robot_idx;
      float inflation_radius;    // max 4 degree;
      bool start_condition;
      int no_targets_count;
      float rotation_w[3];
      float rotation_z[3];
      
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub;
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costMapSub;
      rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_sub;

     

  };
}