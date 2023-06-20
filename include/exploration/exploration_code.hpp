#include <chrono>
#include <memory>
#include <string>

#include <exploration/colored_noise.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
//#include "cartographer_ros_msgs/srv/trajectory_query.hpp"

#include<fstream>
#include<sstream>
#include<iostream>
#include<iomanip>
#include<string>
#include<cstdlib>
#include "std_msgs/msg/bool.h"

#define LARGEST_MAP_DISTANCE 500000 
#define K_ATTRACT 1
#define ROBOT_INTERFERE_RADIUS 50
#define ETA_REPLUSIVE 3
#define DIS_OBTSTACLE 6


using namespace std::chrono_literals;

//#define DEBUG

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
      void mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
      void costmapMergedCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
      void rvizCallBack(const geometry_msgs::msg::PointStamped::SharedPtr msg);
      void dismapConstruction_start_target(int* dismap_, int* dismap_backup_, int* curr, int HEIGHT, int WIDTH);
      void check_clicked_points();
      bool map_data_available();  
      void explore();
      void publish_exploration_state(void);

      nav2_msgs::action::NavigateToPose_Goal robotGoal;
      //geometry_msgs::msg::PoseStamped robotGoal;
      rclcpp::TimerBase::SharedPtr timer_main;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
      size_t count_;


      std::string map_topic,costmap_topic,trajectory_query_name, output_file;
      std::string robot_frame, robot_base_frame;
      int rateHz;  
      std::string  ns, robot_ano_frame_suffix, robot_ano_frame_preffix;
      int rotation_count, n_robot, this_robot_idx;
      float inflation_radius;    // max 4 degree;
      bool start_condition;
      int no_targets_count;
      float rotation_w[3];
      float rotation_z[3];
      std::string* robots_frame_;
      
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub;
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costMapSub;
      rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_sub;

      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub;
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_centroid;

      //rclcpp::Client<cartographer_ros_msgs::srv::TrajectoryQuery>::SharedPtr trajectory_query_client;
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;

      
      double trajectory_length, exploration_time;
      double trajectory_x;
      double trajectory_y;
      
      // tf2_ros::Buffer buffer;
      // tf2_ros::TransformListener listener{buffer};
      std::unique_ptr<tf2_ros::Buffer> buffer;
      std::shared_ptr<tf2_ros::TransformListener> listener{nullptr};

      rclcpp::TimerBase::SharedPtr timer_exploration_state_publisher_;
      std_msgs::msg::Bool sim_exploration_state_;
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sim_exploration_state_publisher_;

      
      
    

  };
}
