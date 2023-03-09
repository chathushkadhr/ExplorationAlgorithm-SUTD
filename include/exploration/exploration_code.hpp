#include <chrono>
#include <memory>
#include <string>

#include <exploration/colored_noise.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_broadcaster.h"
//#include "cartographer_ros_msgs/srv/trajectory_query.hpp"

#include<fstream>
#include<sstream>
#include<iostream>
#include<iomanip>
#include<string>
#include<cstdlib>

#include <list>
#include <mutex>

#define LARGEST_MAP_DISTANCE 500000 
#define K_ATTRACT 1
#define ROBOT_INTERFERE_RADIUS 50


using namespace std::chrono_literals;

//#define DEBUG

namespace exploration{


  class MWFCN_Algo : public rclcpp::Node
  {
    public:
      enum VisualizationType 
      { 
        POINTS = 0,
        LINE = 1
      };

      struct Pixel
      {
        int x = 0;
        int y = 0;

        Pixel(int x, int y): x(x), y(y) {};
        inline bool operator==(const Pixel& p) { return ((p.x == x) && (p.y == y)); };
      };

      struct Cluster
      {
        int x = 0;    // Cluster center x
        int y = 0;    // Cluster center y
        int size = 0; // Number of points in cluster

        Cluster(int x, int y, int size): x(x), y(y), size(size) {};
        Cluster(Pixel point, int size): x(point.x), y(point.y), size(size) {};
        Pixel center() { return Pixel(x, y); };
      };

      MWFCN_Algo();
 
      
    private:
      void timer_callback();
      void debug_param();
      void mapCallBack(const nav_msgs::msg::OccupancyGrid::ConstPtr & msg);
      void costmapMergedCallBack(const nav_msgs::msg::OccupancyGrid::ConstPtr & msg);
      void rvizCallBack(const geometry_msgs::msg::PointStamped::ConstPtr & msg);
      void check_clicked_points();
      bool map_data_available();  
      void explore();

      void set_costmap_data(nav_msgs::msg::OccupancyGrid costmapData);
      nav_msgs::msg::OccupancyGrid get_costmap_data();
      void set_map_data(nav_msgs::msg::OccupancyGrid mapData);
      nav_msgs::msg::OccupancyGrid get_map_data();

      visualization_msgs::msg::Marker create_visualization_msg(int type);

      bool get_transform(std::string target_frame, std::string source_frame, geometry_msgs::msg::TransformStamped &transform);
      std::vector<Cluster> cluster_2D(std::vector<Pixel> points, int proximity_threshold = 10); // changed from 3
      void process_maps(nav_msgs::msg::OccupancyGrid mapData, nav_msgs::msg::OccupancyGrid costmapData, 
                                      std::vector<Pixel> &obstacles, std::vector<Pixel> &targets);
      bool create_potential_map(nav_msgs::msg::OccupancyGrid mapData, Pixel source_point, std::vector<int> &potential_map, int potential_step = 3);
      inline void process_pixel_potential(Pixel source_pixel, 
                                            Pixel target_pixel, 
                                            std::vector<int> &map, 
                                            std::vector<int> &potential_map, 
                                            int map_width, 
                                            std::vector<Pixel> &discovered_pixels,
                                            int unit_potential);
      
      

      nav2_msgs::action::NavigateToPose_Goal robotGoal;
      //geometry_msgs::msg::PoseStamped robotGoal;
      rclcpp::TimerBase::SharedPtr timer_main;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
      size_t count_;

      nav_msgs::msg::OccupancyGrid mapData_,costmapData_;
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
      std::string* robots_base_frame_;
      
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub;
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costMapSub;
      rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_sub;

      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub;
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_centroid;

      //rclcpp::Client<cartographer_ros_msgs::srv::TrajectoryQuery>::SharedPtr trajectory_query_client;
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;

      int goal[2]; //target[2], obstacle[2]
      float  minDis2Frontier;
      std::ifstream infile;

      double trajectory_length, exploration_time;
      double trajectory_x;
      double trajectory_y;
      
      // tf2_ros::Buffer buffer;
      // tf2_ros::TransformListener listener{buffer};
      std::unique_ptr<tf2_ros::Buffer> buffer;
      std::shared_ptr<tf2_ros::TransformListener> listener{nullptr};
      
      std::mutex mtx_map; 
      std::mutex mtx_costmap; 

      const int MAP_PIXEL_OCCUPIED = 100;
      const int MAP_PIXEL_UNKNOWN = -1;
      const int MAP_PIXEL_FREE = 0;

  };
}