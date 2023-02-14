#include <exploration/exploration_code.hpp>

using namespace exploration;
using std::placeholders::_1;

MWFCN_Algo:: MWFCN_Algo() : 
    Node("MWFCN_node"), 
    rotation_count(0),
    start_condition(true),
    no_targets_count(0),
    rotation_w{0.866,  0.500, 1.0},
    rotation_z{0.5  , -0.866, 0.0},
    map_data_received(false),
    init_point_line_frame(false)
{     
    std::string nodename=MWFCN_Algo::get_name();

    this->declare_parameter("map_topic", "/robot1/map");
    map_topic=this->get_parameter("map_topic").get_parameter_value().get<std::string>();
    this->declare_parameter("costmap_topic", "/robot1/move_base/global_costmap/costmap");
    costmap_topic=this->get_parameter("costmap_topic").get_parameter_value().get<std::string>();
    this->declare_parameter("robot_base_frame", "/robot1/base_footprint");
    robot_base_frame=this->get_parameter("robot_base_frame").get_parameter_value().get<std::string>();
    this->declare_parameter("robot_frame", "/robot1/map");
    robot_frame=this->get_parameter("robot_frame").get_parameter_value().get<std::string>();
    this->declare_parameter("namespace", "/robot1");
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
    
    
    // ------------------------------------- subscribe the map topics & clicked points
    tf2_ros::Buffer buffer(this->get_clock());
    tf2_ros::TransformListener listener(buffer);

    // ------------------------------------- publish the detected points for following processing & display
    pub = this->create_publisher<visualization_msgs::msg::Marker>("_shapes", 100);
    pub_centroid = this->create_publisher<visualization_msgs::msg::Marker>("_detected_frontier_centroid", 10);
        
    // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    // timer_ = this->create_wall_timer(
    // 500ms, std::bind(&MWFCN_Algo::timer_callback, this));

    #ifdef DEBUG
    debug_param();
    
    #endif
    rclcpp::Rate rate(rateHz);
    timer_main = this->create_wall_timer( 1s, std::bind(&MWFCN_Algo::explore, this));

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

void MWFCN_Algo::dismapConstruction_start_target(int* dismap_, int* dismap_backup_, int* curr, int HEIGHT, int WIDTH)
{
    std::vector<int *> curr_iter;
    std::vector<int *> next_iter;

    // initialize the dis_map with number of dismapBackup
    memcpy(dismap_, dismap_backup_, sizeof(int) * HEIGHT * WIDTH);

    int iter = 1;
    curr_iter.push_back(new int[2]{curr[0], curr[1]});

    // change pixel of the starting location to -500 to avoid being changed. After processing dismap, we changed it back to 0;
    dismap_[(curr[0]) * WIDTH + curr[1]] = -500;

    while (curr_iter.size() > 0) {
        if(iter>LARGEST_MAP_DISTANCE){
            std::cout << "distance exceeds MAXIMUM SETUP" << std::endl;
            return;
        }
        for (int i = 0; i < curr_iter.size(); i++) {
            if (dismap_[(curr_iter[i][0] + 1) * WIDTH + curr_iter[i][1]] == 0) {
                dismap_[(curr_iter[i][0] + 1) * WIDTH + curr_iter[i][1]] = iter + 3;
                next_iter.push_back(new int[2]{curr_iter[i][0] + 1, curr_iter[i][1]});
            }

            if (dismap_[(curr_iter[i][0]) * WIDTH + curr_iter[i][1] + 1] == 0) {
                dismap_[(curr_iter[i][0]) * WIDTH + curr_iter[i][1] + 1] = iter + 3;
                next_iter.push_back(new int[2]{curr_iter[i][0], curr_iter[i][1] + 1});
            }

            if (dismap_[(curr_iter[i][0] - 1) * WIDTH + curr_iter[i][1]] == 0) {
                dismap_[(curr_iter[i][0] - 1) * WIDTH + curr_iter[i][1]] = iter + 3;
                next_iter.push_back(new int[2]{curr_iter[i][0] - 1, curr_iter[i][1]});
            }

            if (dismap_[(curr_iter[i][0]) * WIDTH + curr_iter[i][1] - 1] == 0) {
                dismap_[(curr_iter[i][0]) * WIDTH + curr_iter[i][1] - 1] = iter + 3;
                next_iter.push_back(new int[2]{curr_iter[i][0], curr_iter[i][1] - 1});
            }

            if (dismap_[(curr_iter[i][0] + 1) * WIDTH + curr_iter[i][1] + 1] == 0) {
                dismap_[(curr_iter[i][0] + 1) * WIDTH + curr_iter[i][1] + 1] = iter + 4;
                next_iter.push_back(new int[2]{curr_iter[i][0] + 1, curr_iter[i][1] + 1});
            }

            if (dismap_[(curr_iter[i][0] + 1) * WIDTH + curr_iter[i][1] - 1] == 0) {
                dismap_[(curr_iter[i][0] + 1) * WIDTH + curr_iter[i][1] - 1] = iter + 4;
                next_iter.push_back(new int[2]{curr_iter[i][0] + 1, curr_iter[i][1] - 1});
            }

            if (dismap_[(curr_iter[i][0] - 1) * WIDTH + curr_iter[i][1] + 1] == 0) {
                dismap_[(curr_iter[i][0] - 1) * WIDTH + curr_iter[i][1] + 1] = iter + 4;
                next_iter.push_back(new int[2]{curr_iter[i][0] - 1, curr_iter[i][1] + 1});
            }

            if (dismap_[(curr_iter[i][0] - 1) * WIDTH + curr_iter[i][1] - 1] == 0) {
                dismap_[(curr_iter[i][0] - 1) * WIDTH + curr_iter[i][1] - 1] = iter + 4;
                next_iter.push_back(new int[2]{curr_iter[i][0] - 1, curr_iter[i][1] - 1});
            }

        }
        for (int curr_idx = 0; curr_idx < curr_iter.size(); curr_idx++){
            delete [] curr_iter[curr_idx];
        }
        curr_iter.swap(next_iter);
        std::vector<int *>().swap(next_iter);
        iter++;
    }
    dismap_[(curr[0]) * WIDTH + curr[1]] = 0;  // int only zero is available
    return ;
}


void MWFCN_Algo::check_map_data(){
    if (!(mapData.data.size() < 1 || costmapData.data.size()<1)){
        map_data_received =true;
    } 
}  

void MWFCN_Algo::check_clicked_points(){
    if (!(points.points.size()<1)){
        clicked_point=true;
        points.points.clear();
        pub->publish(points);
        
    }
    else{
        pub->publish(points);
    }
}  


void MWFCN_Algo::explore(){
    if (!map_data_received){
        check_map_data();
    }

    else{
        if(!(init_point_line_frame)){
        
            init_point_line_frame =true;

            robotGoal.header.frame_id=robot_frame;
            robotGoal.pose.position.z=0;
            robotGoal.pose.orientation.z=1.0;

            // ------------------------------------- initilize the visualized points & lines  
            points.header.frame_id = mapData.header.frame_id;
            points.header.stamp = rclcpp::Time(0);
            points.type 			= points.POINTS;
            points.action           = points.ADD;
            points.pose.orientation.w =1.0;
            points.scale.x 			= 0.3; 
            points.scale.y			= 0.3; 
            points.color.r 			= 1.0;   // 255.0/255.0;
            points.color.g 			= 0.0;   // 0.0/255.0;
            points.color.b 			= 0.0;   // 0.0/255.0;
            points.color.a			= 1.0;
            points.lifetime         = rclcpp::Duration(0,0); // TODO : currently points are stored forever

            line.header.frame_id    = mapData.header.frame_id;
            line.header.stamp       = rclcpp::Time(0);
            line.type				= line.LINE_LIST;
            line.action             = line.ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x 			= 0.03;
            line.scale.y			= 0.03;
            line.color.r			= 1.0;   // 0.0/255.0;
            line.color.g			= 0.0;   // 0.0/255.0;
            line.color.b 			= 1.0;   // 236.0/255.0;
            line.color.a 			= 1.0;
            line.lifetime           = rclcpp::Duration(0,0); // TODO : currently points are stored forever
            
            // -------------------------------------Initialize all robots' frame;
            std::string robots_frame[n_robot];
            for (int i = 1; i < n_robot+1; i++){
                std::stringstream ss;              
                ss << robot_ano_frame_preffix;
                ss << i;
                ss << robot_ano_frame_suffix;
                robots_frame[i-1] = ss.str();
            }


        }


        else{
            


            /* Main While*/
        }



    }
        
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


 

    
