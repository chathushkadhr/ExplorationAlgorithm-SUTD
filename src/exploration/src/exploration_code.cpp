#include <exploration/exploration_code.hpp>

using namespace exploration;
using std::placeholders::_1;

MWFCN_Algo:: MWFCN_Algo() : 
      Node("MWFCN_node"), 
      rotation_count(0),
      start_condition(true),
      no_targets_count(0),
      rotation_w{0.866,  0.500, 1.0},
      rotation_z{0.5  , -0.866, 0.0}
{     
       std::string nodename=MWFCN_Algo::get_name();

      this->declare_parameter("map_topic", "robot1/map");
      map_topic=this->get_parameter("map_topic").get_parameter_value().get<std::string>();
      this->declare_parameter("costmap_topic", "robot1/move_base/global_costmap/costmap");
      costmap_topic=this->get_parameter("costmap_topic").get_parameter_value().get<std::string>();
      this->declare_parameter("robot_base_frame", "robot1/base_footprint");
      robot_base_frame=this->get_parameter("robot_base_frame").get_parameter_value().get<std::string>();
      this->declare_parameter("robot_frame", "robot1/map");
      robot_frame=this->get_parameter("robot_frame").get_parameter_value().get<std::string>();
      this->declare_parameter("namespace", "robot1");
      ns=this->get_parameter("namespace").get_parameter_value().get<std::string>();
      this->declare_parameter("rate", 1);
      rateHz=this->get_parameter("rate").get_parameter_value().get<int>();
      this->declare_parameter("inflation_radius", 6.0);
      inflation_radius=this->get_parameter("inflation_radius").get_parameter_value().get<float>();
      this->declare_parameter("n_robot", 1);
      n_robot=this->get_parameter("n_robot").get_parameter_value().get<int>();
      this->declare_parameter("this_robot_idx", 1);
      this_robot_idx=this->get_parameter("this_robot_idx").get_parameter_value().get<int>();
      this->declare_parameter("robot_ano_frame_preffix", "robot");
      robot_ano_frame_preffix=this->get_parameter("robot_ano_frame_preffix").get_parameter_value().get<std::string>();
      this->declare_parameter("robot_ano_frame_suffix", "base_footprint");
      robot_ano_frame_suffix=this->get_parameter("robot_ano_frame_suffix").get_parameter_value().get<std::string>();
      this->declare_parameter("trajectory_query_name", "robot1/trajectory_query");
      trajectory_query_name=this->get_parameter("trajectory_query_name").get_parameter_value().get<std::string>();
      this->declare_parameter("output_file", "$(find exploration/data/robot1_MWFCN_trajectory.txt");      
      output_file=this->get_parameter("output_file").get_parameter_value().get<std::string>();
      this->declare_parameter("output_map_file", "$(find exploration)/data/robot1_MWFCN_explored_map.txt");
      output_map_file=this->get_parameter("output_map_file").get_parameter_value().get<std::string>();  
      
      //ros::Rate rate(rateHz); To edit

      // ------------------------------------- subscribe the map topics & clicked points
      sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, 10, std::bind(&MWFCN_Algo::mapCallBack, this, _1));
      costMapSub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(costmap_topic, 20, std::bind(&MWFCN_Algo::costmapMergedCallBack, this, _1));
      rviz_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10, std::bind(&MWFCN_Algo::rvizCallBack, this, _1));
      
      
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MWFCN_Algo::timer_callback, this));

      #ifdef DEBUG
      debug_param();
      
      #endif
}



void MWFCN_Algo::mapCallBack(const nav_msgs::msg::OccupancyGrid::ConstPtr & msg)
{
    //_mt.lock();
	mapData=*msg;
    // std::cout << "assigner receives map" << std::endl;
    //_mt.unlock();
}

void MWFCN_Algo::costmapMergedCallBack(const nav_msgs::msg::OccupancyGrid::ConstPtr & msg)
{
    //_mt.lock();
	costmapData=*msg;
    // std::cout << "assigner receives costmap" << std::endl;
    //_mt.unlock();
}


void MWFCN_Algo::rvizCallBack(const geometry_msgs::msg::PointStamped::ConstPtr & msg)
{ 
	p.x=msg->point.x;
	p.y=msg->point.y;
	p.z=msg->point.z;
	points.points.push_back(p);
}



void MWFCN_Algo::debug_param(){
      RCLCPP_INFO_STREAM(MWFCN_Algo::get_logger(), "Map topic: " << map_topic
      <<"\ncostmap_topic: "<<costmap_topic
      <<"\nrobot_base_frame: "<<robot_base_frame
      <<"\nrobot_frame: "<<robot_frame
      <<"\nnamespace: "<<ns
      <<"\nrate: "<<rateHz
      <<"\ninflation_radius: "<<inflation_radius
      <<"\nn_robot: "<<n_robot
      <<"\nthis_robot_idx: "<<this_robot_idx
      <<"\nrobot_ano_frame_preffix: "<<robot_ano_frame_preffix
      <<"\nrobot_ano_frame_suffix: "<<robot_ano_frame_suffix
      <<"\ntrajectory_query_name: "<<trajectory_query_name
      <<"\noutput_file: "<<output_file
      <<"\noutput_map_file: "<<output_map_file
      );


}

void MWFCN_Algo:: timer_callback()
{
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      std::string x = std::to_string(rotation_w[0]);
       RCLCPP_INFO(this->get_logger(), "I'm here");
      publisher_->publish(message);
}


 

    
