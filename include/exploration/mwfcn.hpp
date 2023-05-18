#include <chrono>
#include <string>
#include <list>
#include <mutex>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "exploration/msg/exploration_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include<string>

//#define DEBUG

namespace exploration{
  class MWFCN : public rclcpp::Node
  {
    public:
      enum VisualizationType 
      { 
        POINTS = 0,
        LINE = 1,
        SPHERES = 2
      };

      struct Pixel
      {
        int x = 0;
        int y = 0;

        Pixel() {};
        Pixel(int x, int y): x(x), y(y) {};
        inline bool operator==(const Pixel& p) { return ((p.x == x) && (p.y == y)); };
      };

      struct Cluster
      {
        int x = 0;    // Cluster center x
        int y = 0;    // Cluster center y
        int size = 0; // Number of points in cluster

        Cluster() {};
        Cluster(int x, int y, int size): x(x), y(y), size(size) {};
        Cluster(Pixel point, int size=1): x(point.x), y(point.y), size(size) {};
        Pixel center() { return Pixel(x, y); };
      };

      MWFCN();
 
      
    private:
      void map_callback(const nav_msgs::msg::OccupancyGrid msg);
      void costmap_callback(const nav_msgs::msg::OccupancyGrid msg);
      void enable_exploration_callback(const std_msgs::msg::Bool msg);

      void set_costmap_data(nav_msgs::msg::OccupancyGrid costmapData);
      nav_msgs::msg::OccupancyGrid get_costmap_data();
      void set_map_data(nav_msgs::msg::OccupancyGrid mapData);
      nav_msgs::msg::OccupancyGrid get_map_data();
      bool map_data_available(void);

      void explore();
      bool get_transform(std::string target_frame, std::string source_frame, geometry_msgs::msg::TransformStamped &transform);
      std::vector<Cluster> cluster_2D(std::vector<Pixel> points, int proximity_threshold = 10); // changed from 3
      void find_frontiers(nav_msgs::msg::OccupancyGrid mapData, std::vector<Pixel> &targets);
      std::vector<MWFCN::Pixel> inflate_obstacles(nav_msgs::msg::OccupancyGrid &map, float inflation = 0.3);
      void clear_inflation(nav_msgs::msg::OccupancyGrid &map, Pixel center, float radius = 0.3);
      inline void process_pixel_inflation(Pixel target_pixel, nav_msgs::msg::OccupancyGrid &map, std::list<Pixel> &inflated_pixels, int inflation = 50);
      void copy_obstacles_from_map(nav_msgs::msg::OccupancyGrid &map, nav_msgs::msg::OccupancyGrid obstacle_map, uint8_t min_threshold = 0);
      bool create_potential_map(nav_msgs::msg::OccupancyGrid mapData, Pixel source_point, std::vector<int> &potential_map, int potential_step = 3);
      inline void process_pixel_potential(Pixel source_pixel, 
                                            Pixel target_pixel, 
                                            std::vector<int> &map, 
                                            std::vector<int> &potential_map, 
                                            int map_width, 
                                            std::vector<Pixel> &discovered_pixels,
                                            int unit_potential);
      float calculate_attraction(std::vector<std::vector<int>> robot_potential_maps, int map_width, Cluster target);
      std::map<int, Pixel> find_optimal_targets(std::vector<std::vector<int>> robot_potential_maps, 
                                                std::vector<Cluster> target_clusters, 
                                                nav_msgs::msg::OccupancyGrid map);
      void filter_map_noise(nav_msgs::msg::OccupancyGrid &mapData);
      inline bool is_pixel_occupied(Pixel pixel, nav_msgs::msg::OccupancyGrid map);
      visualization_msgs::msg::Marker create_visualization_msg(int type);
      bool get_ros_parameters(void);
      void publish_exploration_state(void);

      // Parameters
      std::string map_topic_, costmap_topic_; 
      std::string robot_base_frame_, map_frame_, robot_frame_prefix_;
      float rate_;
      float obstacle_inflation_radius_;
      int robot_count_;
      uint robot_id_;
      std::vector<std::string> robot_base_frames_;  // Fully qualified frame names

      // ROS Subscribers, Publishers and Action clients
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber_;
      rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exploration_cmd_subscriber_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr target_publisher_;
      rclcpp::Publisher<exploration::msg::ExplorationState>::SharedPtr exploration_state_publisher_;
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_client_;

      // ROS TF2
      std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

      // Attributes
      rclcpp::TimerBase::SharedPtr timer_main_;
      rclcpp::TimerBase::SharedPtr timer_exploration_state_publisher_;
      // Shared variables
      nav_msgs::msg::OccupancyGrid mapData_, costmapData_;
      nav2_msgs::action::NavigateToPose_Goal robot_goal_;
      exploration::msg::ExplorationState exploration_state_;
      std::mutex mtx_map; 
      std::mutex mtx_costmap; 
      std::mutex mtx_exploration_state;

      // Constants
      const int MAP_PIXEL_OCCUPIED = 100;
      const int MAP_PIXEL_INFLATED = 99;
      const int MAP_PIXEL_UNKNOWN = -1;
      const int MAP_PIXEL_FREE = 0;

      const int LARGEST_MAP_DISTANCE = 500000; 

  };
}