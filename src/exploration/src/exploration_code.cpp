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
    map_data_received(false)
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
    
    robots_frame_ = new std::string[n_robot];

    for (int i = 1; i < n_robot+1; i++){

        std::stringstream ss;              
        ss << robot_ano_frame_preffix;
        ss << i;
        ss << robot_ano_frame_suffix;

        robots_frame_[i-1] = ss.str();
    }

    rclcpp::Rate rate(rateHz);

    // ------------------------------------- subscribe the map topics & clicked points
    sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, 10, std::bind(&MWFCN_Algo::mapCallBack, this, _1));
    costMapSub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(costmap_topic, 20, std::bind(&MWFCN_Algo::costmapMergedCallBack, this, _1));
    rviz_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10, std::bind(&MWFCN_Algo::rvizCallBack, this, _1));
    
    
    // ------------------------------------- subscribe the map topics & clicked points

    // ------------------------------------- publish the detected points for following processing & display
    pub = this->create_publisher<visualization_msgs::msg::Marker>("_shapes", 100);
    pub_centroid = this->create_publisher<visualization_msgs::msg::Marker>("_detected_frontier_centroid", 10);
        
    // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    // timer_ = this->create_wall_timer(
    // 500ms, std::bind(&MWFCN_Algo::timer_callback, this));

    buffer= std::make_unique<tf2_ros::Buffer>(this->get_clock());
    listener= std::make_shared<tf2_ros::TransformListener>(*buffer);

    #ifdef DEBUG
    debug_param();
    
    #endif
    // trajectory_query_client =this->create_client<cartographer_ros_msgs::srv::TrajectoryQuery>(trajectory_query_name);
    // trajectory_query_client->wait_for_service();

    timer_main = this->create_wall_timer( 1s, std::bind(&MWFCN_Algo::explore, this));  //TODO adjust the period and return part at line 403

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
        if(!clicked_point){
            check_clicked_points();
        }
        else{
            //Main While Loop

             // ---------------------------------------- variables from ROS input;
            HEIGHT = mapData.info.height;
            WIDTH  = mapData.info.width;
            
            
            int* dismap_backup = new int[HEIGHT*WIDTH];
            int map[HEIGHT*WIDTH];

            // ---------------------------------------- initialize the map & dismap
            for (int i=0; i<HEIGHT; i++)
            {
                for (int j=0; j<WIDTH; j++)
                {
                    map[i*WIDTH + j] = (int) mapData.data[i*mapData.info.width + j];
                    dismap_backup[i*WIDTH + j] = map[i*WIDTH + j];
                }
            }

            // ------------------------------------------ find the obstacles & targets
            for (int i = 2; i < HEIGHT-2; i++){
                for (int j = 2; j < WIDTH-2; j++){
                    if(map[i*WIDTH + j] == 100){
                        obstacles.push_back(new int[2]{i,j});
                    }
                    else if(map[i*WIDTH + j] == -1){
                        // accessiable frontiers
                        int numFree = 0, temp1 = 0;

                        if (map[(i + 1)*WIDTH + j] == 0){
                            temp1 += (map[(i + 2)*WIDTH + j    ] == 0) ? 3 : 0;
                            temp1 += (map[(i + 1)*WIDTH + j + 1] == 0) ? 3 : 0;
                            temp1 += (map[(i + 1)*WIDTH + j - 1] == 0) ? 3 : 0;
                            temp1 += (map[(i + 2)*WIDTH + j + 1] == 0) ? 4 : 0;
                            temp1 += (map[(i + 2)*WIDTH + j - 1] == 0) ? 4 : 0;
                            temp1 += (map[      i*WIDTH + j + 1] == 0) ? 4 : 0;
                            temp1 += (map[      i*WIDTH + j - 1] == 0) ? 4 : 0;
                            numFree += (temp1 > 0);
                        }

                        if (map[i*WIDTH + j + 1] == 0){
                            temp1 = 0;
                            temp1 += (map[      i*WIDTH + j + 2] == 0) ? 3 : 0;
                            temp1 += (map[(i + 1)*WIDTH + j + 1] == 0) ? 3 : 0;
                            temp1 += (map[(i - 1)*WIDTH + j + 1] == 0) ? 3 : 0;
                            temp1 += (map[(i + 1)*WIDTH + j + 2] == 0) ? 4 : 0;
                            temp1 += (map[(i + 1)*WIDTH + j    ] == 0) ? 4 : 0;
                            temp1 += (map[(i - 1)*WIDTH + j + 2] == 0) ? 4 : 0;
                            temp1 += (map[(i - 1)*WIDTH + j    ] == 0) ? 4 : 0;
                            numFree += (temp1 > 0);
                        }

                        if (map[(i - 1) *WIDTH + j] == 0){
                            temp1 = 0;
                            temp1 += (map[(i - 1)*WIDTH + j + 1] == 0) ? 3 : 0;
                            temp1 += (map[(i - 1)*WIDTH + j - 1] == 0) ? 3 : 0;
                            temp1 += (map[(i - 2)*WIDTH + j    ] == 0) ? 3 : 0;
                            temp1 += (map[      i*WIDTH + j + 1] == 0) ? 4 : 0;
                            temp1 += (map[      i*WIDTH + j - 1] == 0) ? 4 : 0;
                            temp1 += (map[(i - 2)*WIDTH + j + 1] == 0) ? 4 : 0;
                            temp1 += (map[(i - 2)*WIDTH + j - 1] == 0) ? 4 : 0;
                            numFree += (temp1 > 0);
                        }

                        if (map[i * WIDTH + j - 1] == 0){
                            temp1 = 0;
                            temp1 += (map[    i  *WIDTH + j - 2] == 0) ? 3 : 0;
                            temp1 += (map[(i + 1)*WIDTH + j - 1] == 0) ? 3 : 0;
                            temp1 += (map[(i - 1)*WIDTH + j - 1] == 0) ? 3 : 0;
                            temp1 += (map[(i + 1)*WIDTH + j    ] == 0) ? 4 : 0;
                            temp1 += (map[(i + 1)*WIDTH + j - 2] == 0) ? 4 : 0;
                            temp1 += (map[(i - 1)*WIDTH + j    ] == 0) ? 4 : 0;
                            temp1 += (map[(i - 1)*WIDTH + j - 2] == 0) ? 4 : 0;
                            numFree += (temp1 > 0);
                        }

                        if( numFree > 0 ) {
                            targets.push_back(new int[2]{i,j});
                        }
                    }
                }
            }

            {
                for (int idx_target = targets.size()-1; idx_target >= 0; idx_target--) {
                    
                    float loc_x = targets[idx_target][1]*mapData.info.resolution + mapData.info.origin.position.x;
                    float loc_y = targets[idx_target][0]*mapData.info.resolution + mapData.info.origin.position.y;
                    int index_costmap = (loc_y - costmapData.info.origin.position.y)/costmapData.info.resolution * costmapData.info.width + (loc_x - costmapData.info.origin.position.x)/costmapData.info.resolution;
                    if (costmapData.data[index_costmap] >0){
                        targets.erase(targets.begin() + idx_target);
                        continue;
                    }
                }
                std::cout << "number targets after erase (costmap): " << targets.size() << std::endl;
            }
            
            // ------------------------------------------ remove targets within the inflation radius of obstacles.
            {
                for(int idx_target = targets.size()-1; idx_target>=0; idx_target--) {
                    for (int i = 0; i < obstacles.size(); i++) {
                        if (abs(targets[idx_target][0] - obstacles[i][0]) +
                            abs(targets[idx_target][1] - obstacles[i][1]) < inflation_radius) {
                            targets.erase(targets.begin() + idx_target);
                            break;
                        }
                    }
                std::cout << "number targets after erase (obstacles): " << targets.size() << std::endl;
                }
            }

            // ------------------------------------------ exploration finish detection
            if(targets.size() == 0){
                if(no_targets_count == 8){
                    std::cout << "exploration done" << std::endl;
                    std::vector<geometry_msgs::msg::PointStamped> path_list;

                    // auto request = std::make_shared<cartographer_ros_msgs::srv::TrajectoryQuery::Request>();
                    // request->trajectory_id = 0;
                    // auto result = trajectory_query_client->async_send_request(request);
                    // exploration_time = result.get()->trajectory[0].header.stamp.sec;
                    // exploration_time = result.get()->trajectory.back().header.stamp.sec - exploration_time;
                    //std::cout <<  ns << "exploration_time is:" << exploration_time << " seconds" << std::endl;

                    // std::ofstream ofile(output_file);

                    // trajectory_x = result.get()->trajectory[0].pose.position.x;
                    // trajectory_y = result.get()->trajectory[0].pose.position.y;
                                    
                    // ofile << "[";
                    // ofile << trajectory_x << "," << trajectory_y << std::endl;
                    // for (int i = 1; i < result.get()->trajectory.size(); i++){
                    //     double temp_x = result.get()->trajectory[i].pose.position.x;
                    //     double temp_y = result.get()->trajectory[i].pose.position.y;
                    //     ofile << temp_x  << ", " <<  temp_y << ";" << std::endl;
                    //     double delta_x = trajectory_x - temp_x;
                    //     double delta_y = trajectory_y - temp_y;
                    //     trajectory_length += sqrt(delta_x*delta_x + delta_y*delta_y);
                    //     trajectory_x = temp_x;
                    //     trajectory_y = temp_y; 
                    // }
                    // ofile << "]" << std::endl;
                    // ofile.close();
                    //std::cout <<  ns << "exploration trajectory length = " << trajectory_length << " meters" << std::endl;
                    
                    std::ofstream ofile2(output_map_file);
                    ofile2 <<  ns << "map Origin (" << mapData.info.origin.position.x << " ," << mapData.info.origin.position.y << ")" << std::endl;
                    for(int i = 0; i < mapData.data.size(); i++){
                        ofile2 << mapData.data[i] << " ";
                    }
                    ofile2.close();
                    timer_main->cancel();
                }   

                no_targets_count ++;
                std::cout << ns << "no targets count = " << no_targets_count << std::endl;
                //
                return;
            }
            else{
                no_targets_count = 0;
            }
            // ---------------------------------------- define the current point;
            geometry_msgs::msg::TransformStamped transform;
            int  temp=0;
            while (temp==0){
                try{
                    temp=1;
                    transform = buffer->lookupTransform(mapData.header.frame_id,robot_base_frame,tf2::TimePointZero);
                }
                catch( const tf2::TransformException & ex){
                    temp=0;
                    rclcpp::sleep_for(100ms);
                }
            }
            currentLoc[0] = floor((transform.transform.translation.y  -mapData.info.origin.position.y)/mapData.info.resolution);
            currentLoc[1] = floor((transform.transform.translation.x  -mapData.info.origin.position.x)/mapData.info.resolution);
            path.push_back( currentLoc );

            for(int i = 0; i< obstacles.size(); i++){
                dismap_backup[(obstacles[i][0])*WIDTH + obstacles[i][1]] = -2;
            }

            // ------------------------------------------ cluster targets into different groups and find the center of each group.
            // Note: x & y value of detected targets are in increasing order because of the detection is in laser scan order.
            std::vector<int* > target_process(targets);
            std::vector<int* > cluster_center;
            std::vector<int>   infoGain_cluster;

            while(target_process.size() > 0){
                std::vector<int* > target_cluster;
                target_cluster.push_back(target_process.back());
                target_process.pop_back();

                bool condition = true;
                while(condition){
                    condition = false;
                    int size_target_process = target_process.size();
                    for (int i = size_target_process-1; i >= 0 ; i--){
                        for (int j = 0; j < target_cluster.size(); j++){
                            int dis_ = abs(target_process[i][0] - target_cluster[j][0]) +  abs(target_process[i][1] - target_cluster[j][1]);
                            if(dis_ < 3){
                                target_cluster.push_back(target_process[i]);
                                target_process.erase(target_process.begin() + i);
                                condition = true;
                                break;
                            }
                        }
                    }
                }

                int center_[2]={0, 0};
                int num_ = target_cluster.size();
                for(int i = 0; i < num_; i++){
                    center_[0] += target_cluster[i][0];
                    center_[1] += target_cluster[i][1];
                }

                float center_float[2] = {float(center_[0]), float(center_[1])};
                center_float[0] = center_float[0]/float(num_);
                center_float[1] = center_float[1]/float(num_);

                float min_dis_ = 100.0;
                int min_idx_   = 10000;
                for(int i = 0; i < num_; i++){
                    float temp_dis_ = abs(center_float[0]-float(target_cluster[i][0])) + abs(center_float[1]-float(target_cluster[i][1]));
                    if(temp_dis_ < min_dis_){
                        min_dis_ = temp_dis_;
                        min_idx_ = i;
                    }
                }

                cluster_center.push_back(new int[2]{target_cluster[min_idx_][0], target_cluster[min_idx_][1]});
                infoGain_cluster.push_back(num_);
            }

            // ------------------------------------------ Display Cluster centroids
            points.points.clear();
            for(int i = 0; i < cluster_center.size(); i++){
                geometry_msgs::msg::Point temp;
                temp.x = cluster_center[i][1] * mapData.info.resolution + mapData.info.origin.position.x;
                temp.y = cluster_center[i][0] * mapData.info.resolution + mapData.info.origin.position.y;
                temp.z = 0;
                points.points.push_back(temp);
            }
            pub_centroid->publish(points);
            
            // ------------------------------------------ Generate Dismap starting from targets
            int cluster_num = cluster_center.size();
            int** dismap_target = new int* [cluster_num];
            for (int i = 0; i<cluster_num; i++){
                dismap_target[i] = new int[HEIGHT*WIDTH];
            }

            for(int i = 0; i< cluster_num; i++) {
                dismapConstruction_start_target(dismap_target[i], dismap_backup, cluster_center[i], HEIGHT, WIDTH);   // dismap_target: target-generate potential dismap_backup: map info cluster_center: target pos 
                dismap_targets_ptr.push_back(dismap_target[i]);
            }

            // ------------------------------------------ receive robots' locations
            bool ifmapmerged_vec[n_robot];
            geometry_msgs::msg::TransformStamped transform_robot[n_robot];
            
            for(int i = 0; i < n_robot; i++){

                ifmapmerged_vec[i] = false;
                if(i == this_robot_idx-1){
                    continue;
                }

                geometry_msgs::msg::TransformStamped transform_;

                try{
                    transform = buffer->lookupTransform(robot_frame, robots_frame_[i],tf2::TimePointZero);
                }
                catch( const tf2::TransformException & ex){
                    temp=0;
                    rclcpp::sleep_for(100ms);
                }

        

                //std::cout<< robot_frame << " receive " << robots_frame[i] << " (" << transform_.getOrigin().getX() << ", "<<   transform_.getOrigin().getY()<< " )" << std::endl;
                transform_robot[i] = transform_;
                ifmapmerged_vec[i] = true;
                //std::cout<< robot_frame << " receive " << robots_frame[i] << " (" << transform_robot[i].getOrigin().getX() << ", "<<   transform_robot[i].getOrigin().getY()<< " )" << std::endl;

            }

            
            // ------------------------------------------ calculate path.
            int iteration = 1;
            int currentPotential = 10000;
            int riverFlowPotentialGain = 1;
            minDis2Frontier  = 10000;  // a random initialized value greater than all possible distances.
            //std::random_device rd{};
            //std::mt19937 gen{rd()};
            //std::normal_distribution<> d{0,0.01};
            //std::map<int, int> hist{};

            // ------------------------------------------ Colored noise (White: alpha = 0.0, Pink: alpha = 1.0, Brown: alpha = 2.0)
            float sigma = 0.6; //for the potential function
            double alpha = 2;
            //int n = n_robot;
            double q_d = 0.095;
            int seed = 0;
            double *noise;

            while(iteration < 3000 && minDis2Frontier > 1){
                // ------------------------------------------
                // ------------------------------------------
                // ------------------------------------------ get the minimial potential of the points around currentLoc
                {
                    // ------------------------------------------ put locations around the current location into loc_around
                    float potential[8];
                    int min_idx = -1;
                    float min_potential = 10000;
                    int* loc_around[8];

                    // up
                    loc_around[0] = new int[2]{currentLoc[0]    , currentLoc[1] + 1};
                    // up-left
                    loc_around[1] = new int[2]{currentLoc[0] - 1, currentLoc[1] + 1};
                    // left
                    loc_around[2] = new int[2]{currentLoc[0] - 1, currentLoc[1]};
                    // down-left
                    loc_around[3] = new int[2]{currentLoc[0] - 1, currentLoc[1] - 1};
                    // down
                    loc_around[4] = new int[2]{currentLoc[0]    , currentLoc[1] - 1};
                    // down-right
                    loc_around[5] = new int[2]{currentLoc[0] + 1, currentLoc[1] - 1};
                    // right
                    loc_around[6] = new int[2]{currentLoc[0] + 1, currentLoc[1]};
                    // up-right
                    loc_around[7] = new int[2]{currentLoc[0] + 1, currentLoc[1] + 1};

                    

                    // ------------------------------------------ calculate potentials of four neighbors of currentLoc
                    for (int i = 0; i < 8; i++){
                        int curr_around[2]={loc_around[i][0], loc_around[i][1]};

                        { // ------------------------------------ calculate current potential
                            float attract = 0, repulsive = 0;
                            for (int j = 0; j < cluster_center.size(); j++){
                                // int temp_int = dismap_targets_ptr[j][(curr_around[0])*WIDTH + curr_around[1]];
                                float temp = float(dismap_targets_ptr[j][(curr_around[0])*WIDTH + curr_around[1]]);
                                if(temp < 1){
                                    // std::cout << "zero loc: (" <<  cluster_center[j][0]   << ", " <<  cluster_center[j][1] << ")" << " temp" << temp << std::endl;
                                    // std::cout << "curr loc: (" <<  curr_around[0]  << ", " << curr_around[1] << ")" << std::endl;
                                    continue;
                                }
                                attract     = attract - K_ATTRACT*infoGain_cluster[j]/temp;
                            }

                            // there is no need to calculate potential of obstacles because information of obstacles have already been encoded in wave-front distance.
                            // for (int j = 0; j < obstacles.size(); j++){
                            //     float dis_obst = abs(obstacles[j][0]- curr_around[0]) + abs(obstacles[j][1]- curr_around[1]);
                            //     if( dis_obst <= DIS_OBTSTACLE) {
                            //         float temp = (1 / dis_obst - 1 / DIS_OBTSTACLE);
                            //         repulsive = repulsive + 0.5 * ETA_REPLUSIVE * temp * temp;
                            //     }
                            // }

                            // to increase the potential if currend point has been passed before
                            for (int j =0; j < path.size(); j++){
                                if(curr_around[0] == path[j][0] && curr_around[1] == path[j][1]){
                                    attract += riverFlowPotentialGain*5;
                                }
                            }
    

                            // Add impact of robots.
                            for(int i = 0; i < n_robot; i++){
                                if(ifmapmerged_vec[i] ){
                                    int index_[2] = {int(round((transform_robot[i].transform.translation.y- mapData.info.origin.position.y)/mapData.info.resolution)), int(round((transform_robot[i].transform.translation.x - mapData.info.origin.position.x)/mapData.info.resolution))};
                                    int dis_ = abs(curr_around[0] - index_[0]) + abs(curr_around[1] - index_[1]);
                                    //float sample = d(gen);
                                    seed++;
                                    noise = f_alpha ( n_robot, q_d, alpha, &seed );
                                    if( dis_ < ROBOT_INTERFERE_RADIUS){
                                        float temp_ = exp((dis_ - ROBOT_INTERFERE_RADIUS)/sigma) + noise[i];
                                        attract += temp_;
                                        std::cout << "sigma: " << sigma << std::endl;
                                        std::cout << "noise: " << noise[i] << std::endl;
                                        std::cout << "robot" << i+1 << " loc  :( " << index_[0] << ", " << index_[1] << ")" << std::endl;
                                        std::cout << ns << " loc  :( " << curr_around[0] << ", " << curr_around[1] << ")" << std::endl;
                                        std::cout << robot_frame << " add " << robots_frame_[i] <<"'s potential = " << temp_ << std::endl;
                                    }
                                }
                            }

                            // potential[i] = attract + repulsive;
                            
                            potential[i] = attract;
                            if(min_potential > potential[i] ){
                                min_potential = potential[i];
                                min_idx = i;
                            }
                        }
                    }
                    if(currentPotential > min_potential){
                        path.push_back(loc_around[min_idx]);
                        currentPotential = min_potential;
                        
                    }
                    else{
                        riverFlowPotentialGain++;
                    }

                    for(int del_idx = 0; del_idx <8 ; del_idx++){
                        if(del_idx != min_idx){
                            delete [] loc_around[del_idx];
                        }
                    }
                }

                currentLoc[0] = (path.back())[0];
                currentLoc[1] = (path.back())[1];
                
                for (int i = 0; i < cluster_center.size() ; i++){
                    int temp_dis_ =  dismap_targets_ptr[i][(currentLoc[0])*WIDTH + currentLoc[1]];
                    if( (temp_dis_ == 0) && (abs(currentLoc[0]-cluster_center[i][0]) + abs(currentLoc[1]-cluster_center[i][1])) > 0){
                        continue;
                    }

                    if(minDis2Frontier > temp_dis_ ){
                        minDis2Frontier = temp_dis_;
                    }
                }
                iteration++;

                // ---------------------------------------- publish path for displaying in rviz
                if(iteration >= 1){
                    p.x=(path[path.size()-2])[1] * mapData.info.resolution + mapData.info.origin.position.x; 
                    p.y=(path[path.size()-2])[0] * mapData.info.resolution + mapData.info.origin.position.y;
                    p.z=0.0;
                    line.points.push_back(p);
                    p.x=currentLoc[1] * mapData.info.resolution + mapData.info.origin.position.x;
                    p.y=currentLoc[0] * mapData.info.resolution + mapData.info.origin.position.y;
                    p.z=0.0;
                    line.points.push_back(p);
                    pub->publish(line); 
                }
            }

            goal[0] = path.back()[0];
            goal[1] = path.back()[1];
        
            if(start_condition){
                geometry_msgs::msg::TransformStamped  transform;
                int  temp=0;
                while (temp==0){
                    try{
                    temp=1;
                    transform = buffer->lookupTransform(mapData.header.frame_id,robot_base_frame,tf2::TimePointZero);
                    }
                    catch( const tf2::TransformException & ex){
                        temp=0;
                        rclcpp::sleep_for(100ms);
                    }
                }
                int loc_x = transform.transform.translation.x;
                int loc_y = transform.transform.translation.y;

                robotGoal.pose.orientation.z = rotation_z[rotation_count];
                robotGoal.pose.orientation.w = rotation_w[rotation_count];
    
                robotGoal.pose.position.x = loc_x + 0.2;
                robotGoal.pose.position.y = loc_y + 0.2;
            
                start_condition = false;
            }
            else{
                robotGoal.pose.orientation.z = 1;
                robotGoal.pose.orientation.w = 0;
                robotGoal.pose.position.x = goal[1]*mapData.info.resolution + mapData.info.origin.position.x;
                robotGoal.pose.position.y = goal[0]*mapData.info.resolution + mapData.info.origin.position.y;
                robotGoal.header.stamp    = rclcpp::Time(0);
            
                //ac.sendGoal(robotGoal);
            }
            line.points.clear();

            // ------------------------------------------- clear memory
            delete [] dismap_backup;
            for (int i = 0; i<cluster_num; i++){
                delete []  dismap_target[i];
            }
            delete [] dismap_target;

            // ------------------------------------------- keep frequency stable
            // _mt.unlock();
        } //Main while loop scope
        

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
