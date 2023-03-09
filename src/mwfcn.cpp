#include <exploration/mwfcn.hpp>

using namespace exploration;
using std::placeholders::_1;

MWFCN:: MWFCN() : 
    Node("MWFCN_node")
{   
    /*------- Fetch parameters ------*/
    if (!get_ros_parameters()) return; // Exit if parameters fetching failure

    /*------- Create Subscribers and publishers ------*/
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic_, 10, std::bind(&MWFCN::map_callback, this, _1));
    costmap_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(costmap_topic_, 20, std::bind(&MWFCN::costmap_callback, this, _1));
    target_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontier_targets", 1);
    for (int i = 1; i <= robot_count_; i++)
    {
        potential_map_publishers_.push_back(this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                                            robot_frame_prefix_ + std::to_string(i) + "/potential_map", 2));
    }

    /*------- Create navigation_stack action client ------*/
    navigation_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    
    /*------- Initialize TF listener ------*/
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    /*------- Create main callback timer ------*/
    timer_main_ = this->create_wall_timer( std::chrono::duration<double>( 1.0 / rate_ ), std::bind(&MWFCN::explore, this));
}

void MWFCN::explore(){
    
    /*------- Return if maps are not available ------*/
    if (!map_data_available()) return;

    /*------- Return if self map to base_foot_print transform is not available ------*/
    geometry_msgs::msg::TransformStamped map_to_baseframe;
    if (!get_transform(robot_frame, robot_base_frame, map_to_baseframe)) return;

    /*-------  Fetch external variables------*/
    nav_msgs::msg::OccupancyGrid mapData = get_map_data();  
    nav_msgs::msg::OccupancyGrid costmapData = get_costmap_data(); 

    /*------- Extract obstacles and exploration targets from maps ------*/
    std::vector<Pixel> obstacles;
    std::vector<Pixel> targets;
    process_maps(mapData, costmapData, obstacles, targets);
    RCLCPP_INFO_STREAM(this->get_logger(), "Targets count: " << std::to_string(targets.size()));

    /*-------Check exploration stop condition ------*/
    if (targets.size() == 0)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Exploration done!");
        return;
    }

    /*------- Cluster targets into different groups and find the center of each group   ------*/
    std::vector<Cluster> target_clusters = cluster_2D(targets);
    // Display clusters
    visualization_msgs::msg::Marker target_cluster_markers = create_visualization_msg(POINTS);
    for (auto &cluster : target_clusters)
    {
        target_cluster_markers.points.push_back(geometry_msgs::msg::Point().set__x(
            cluster.x * mapData.info.resolution + mapData.info.origin.position.x).set__y(
            cluster.y * mapData.info.resolution + mapData.info.origin.position.y));
    }
    pub_centroid->publish(target_cluster_markers);

    /*------- Initialize the distance map (potential map) ------*/
    std::vector<int> dismap_backup(mapData.data.begin(), mapData.data.end());
    std::vector<std::vector<int>> target_distance_map;
    target_distance_map.reserve(targets.size());
    for (auto &cluster : target_clusters)
    {
        std::vector<int> distance_map;
        create_potential_map(mapData, cluster.center(), distance_map);
        target_distance_map.push_back(distance_map);
    }

    
    /*------- Receive other robots' locations ------*/
    std::vector<bool> robot_available;
    std::vector<geometry_msgs::msg::TransformStamped> robot_transform;
    robot_available.resize(n_robot);
    robot_transform.resize(n_robot);

    for (int i = 0; i < n_robot; i++)
    {
        robot_available[i] = get_transform(robot_frame, robots_base_frame_[i], robot_transform[i]);
    }

    // ---------------------------------------- define the current point;
    int currentLoc[2];
    currentLoc[1] = floor((map_to_baseframe.transform.translation.y  -mapData.info.origin.position.y)/mapData.info.resolution);
    currentLoc[0] = floor((map_to_baseframe.transform.translation.x  -mapData.info.origin.position.x)/mapData.info.resolution);
    


    // ------------------------------------------ calculate path.
    std::vector<Pixel> path;
    int iteration = 1;
    int currentPotential = 10000;
    int riverFlowPotentialGain = 1;
    minDis2Frontier  = 10000;  // a random initialized value greater than all possible distances.
    path.emplace_back( currentLoc[0], currentLoc[1] );
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
    
    //iteration < 3000
    while(iteration < 3000 && minDis2Frontier > 1){
        // ------------------------------------------
        // ------------------------------------------
        // ------------------------------------------ get the minimial potential of the points around currentLoc
        {
            // ------------------------------------------ put locations around the current location into loc_around
            float potential[8];
            int min_idx = -1;
            float min_potential = 10000;
            int loc_around[8][2] = {
                {currentLoc[0]    , currentLoc[1] + 1}, // up
                {currentLoc[0] - 1, currentLoc[1] + 1}, // up-left
                {currentLoc[0] - 1, currentLoc[1]}    , // down
                {currentLoc[0] - 1, currentLoc[1] - 1}, // down-left
                {currentLoc[0]    , currentLoc[1] - 1}, // down
                {currentLoc[0] + 1, currentLoc[1] - 1}, // down-right
                {currentLoc[0] + 1, currentLoc[1]}    , // right
                {currentLoc[0] + 1, currentLoc[1] + 1} // up-right
            }; 

            // ------------------------------------------ calculate potentials of four neighbors of currentLoc
            for (int i = 0; i < 8; i++){
                int curr_around[2]={loc_around[i][0], loc_around[i][1]};
                // Map bounds check
                if ( (curr_around[0] < 0) || (curr_around[0] > mapData.info.width) ||
                     (curr_around[1] < 0) || (curr_around[1] > mapData.info.height) )
                {
                    continue;
                }

                { // ------------------------------------ calculate current potential
                    float attract = 0, repulsive = 0;
                    for (int j = 0; j < target_clusters.size(); j++){
                        float temp = float(target_distance_map[j][(curr_around[1])*mapData.info.width + curr_around[0]]);
                        if(temp < 1){
                            // std::cout << "zero loc: (" <<  cluster_center[j][0]   << ", " <<  cluster_center[j][1] << ")" << " temp" << temp << std::endl;
                            // std::cout << "curr loc: (" <<  curr_around[0]  << ", " << curr_around[1] << ")" << std::endl;
                            continue;
                        }
                        attract     = attract - K_ATTRACT*target_clusters[j].size/temp;
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
                        if(curr_around[0] == path[j].x && curr_around[1] == path[j].y){
                            attract += riverFlowPotentialGain*5;
                        }
                    }

                    // Add impact of robots.
                    for(int i = 0; i < n_robot; i++){
                        if(robot_available[i] ){
                            int index_[2] = {int(round((robot_transform[i].transform.translation.x- mapData.info.origin.position.x)/mapData.info.resolution)), int(round((robot_transform[i].transform.translation.y - mapData.info.origin.position.y)/mapData.info.resolution))};
                            int dis_ = abs(curr_around[0] - index_[0]) + abs(curr_around[1] - index_[1]);
                            //float sample = d(gen);
                            seed++;
                            noise = f_alpha ( n_robot, q_d, alpha, &seed );
                            if( dis_ < ROBOT_INTERFERE_RADIUS){
                                float temp_ = exp((dis_ - ROBOT_INTERFERE_RADIUS)/sigma) + noise[i];
                                attract += temp_;
                                //TODO uncomment 
                                // std::cout << "sigma: " << sigma << std::endl;
                                // std::cout << "noise: " << noise[i] << std::endl;
                                // std::cout << "robot" << i+1 << " loc  :( " << index_[0] << ", " << index_[1] << ")" << std::endl;
                                // std::cout << ns << " loc  :( " << curr_around[0] << ", " << curr_around[1] << ")" << std::endl;
                                // std::cout << robot_frame << " add " << robots_base_frame_[i] <<"'s potential = " << temp_ << std::endl; 
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
            if( currentPotential > min_potential ){
                Pixel min_potential_location(loc_around[min_idx][0], loc_around[min_idx][1]);
                if (!(path.back() == min_potential_location))
                {
                    path.push_back(min_potential_location);
                }
                currentPotential = min_potential;
                
            }
            else{
                riverFlowPotentialGain++;
            }
        }

        currentLoc[0] = (path.back()).x;
        currentLoc[1] = (path.back()).y;
        
        for (int i = 0; i < target_clusters.size() ; i++){
            int temp_dis_ =  target_distance_map[i][(currentLoc[1])*mapData.info.width + currentLoc[0]];
            if( (temp_dis_ == 0) && (abs(currentLoc[0]-target_clusters[i].x) + abs(currentLoc[1]-target_clusters[i].y)) > 0){
                continue;
            }

void MWFCN::map_callback(const nav_msgs::msg::OccupancyGrid msg)
{
    set_map_data(*msg);
}

void MWFCN::costmap_callback(const nav_msgs::msg::OccupancyGrid msg)
{ 
    set_costmap_data(msg);
}

void MWFCN::set_costmap_data(nav_msgs::msg::OccupancyGrid costmapData)
{
    std::unique_lock<std::mutex> lck (mtx_costmap);
    costmapData_=costmapData;
}

nav_msgs::msg::OccupancyGrid MWFCN::get_costmap_data()
{
    std::unique_lock<std::mutex> lck (mtx_costmap);
    return costmapData_;
}

void MWFCN::set_map_data(nav_msgs::msg::OccupancyGrid mapData)
{
    std::unique_lock<std::mutex> lck (mtx_map);
    mapData_=mapData;
}

nav_msgs::msg::OccupancyGrid MWFCN::get_map_data()
{
    std::unique_lock<std::mutex> lck (mtx_map);
    return mapData_;
}

/**
 * @brief Given two frames, finds the transform from source to target frame
 * 
 * @param target_frame 
 * @param source_frame 
 * @param transform 
 * @return true     If transform is available
 * @return false    If transform is not available
 */
bool MWFCN::get_transform(std::string target_frame, std::string source_frame, geometry_msgs::msg::TransformStamped &transform)
{
    try{
        transform = tf_buffer_->lookupTransform(target_frame, source_frame,tf2::TimePointZero);
        return true;
    }
    catch( const tf2::TransformException & ex){
        RCLCPP_WARN_STREAM(this->get_logger(), source_frame + " to " + target_frame + " transform is not available : " + ex.what());
    }
    return false;
}


/**
 * @brief Given a set of 2D points, returns a set of clusters
 * 
 * @param points 
 * @param proximity_threshold (optional) Maximum distance between two points, for them to be considered in one cluster (default = 3)
 * @return std::vector<MWFCN_Algo::Cluster> A vector of Clusters : [cluster center(x,y), number of points within the cluster]
 */
std::vector<MWFCN_Algo::Cluster> MWFCN_Algo::cluster_2D(std::vector<MWFCN_Algo::Pixel> points, int proximity_threshold)
{
    // Note: x & y value of detected targets are in increasing order because of the detection is in laser scan order.
    std::vector<Cluster> clusters;
    clusters.reserve(points.size());

    std::list<Pixel> point_list(points.begin(), points.end());
    std::list<std::list<Pixel>> point_clusters; // List of point clusters (A single point cluster is a list of pixel points)

    do
    {
        /*-------  Create a new cluster ------*/
        point_clusters.emplace_back();
        point_clusters.back().push_back(point_list.back());
        point_list.pop_back();
        double center_x, center_y;

        /*------- Traverse current cluster while checking points in close proximity  ------*/
        for (auto clustered_point=point_clusters.back().begin(); clustered_point!=point_clusters.back().end(); clustered_point++)
        {
            for (auto unclustered_point=point_list.begin(); unclustered_point!=point_list.end();)
            {
                if ((abs(clustered_point->x - unclustered_point->x) + abs(clustered_point->y - unclustered_point->y)) < proximity_threshold)
                {
                    // Found unclustered point closer to current cluster. Move point to cluster
                    point_clusters.back().emplace_back(*unclustered_point);
                    // Add to cluster center value total
                    center_x += unclustered_point->x;
                    center_y += unclustered_point->y;
                    // Erase point from unclustered list and get iterator to next unclustered point
                    unclustered_point = point_list.erase(unclustered_point);
                }
                else
                {
                    unclustered_point++;
                }
            }
        }

        /*-------  Calculate cluster center and find data point closest to center ------*/
        center_x /= point_clusters.back().size();
        center_y /= point_clusters.back().size();

        /*------- Find closest data point to cluster center  ------*/
        float min_dist = 100.0;
        Pixel center_point(0,0);
        for (auto &point : point_clusters.back())
        {
            if( (abs(point.x - center_x) + abs(point.y - center_y) ) < min_dist)
            {
                min_dist = abs(point.x - center_x) + abs(point.y - center_y);
                center_point = point;
            }
        }

        /*------- Add cluster data to cluster vector  ------*/
        clusters.emplace_back(center_point, point_clusters.back().size());

    } while (!point_list.empty());
    
    return clusters;
}

/**
 * @brief Extracts obstacle points and exploration targets given a robot map and costmap
 * 
 * @param map 
 * @param costmap 
 * @param obstacles 
 * @param targets 
 */
void MWFCN_Algo::process_maps(nav_msgs::msg::OccupancyGrid mapData, nav_msgs::msg::OccupancyGrid costmapData, 
                                      std::vector<Pixel> &obstacles, std::vector<Pixel> &targets)
{
     /*------- Initialize the map ------*/
    int map_height = mapData.info.height;
    int map_width = mapData.info.width;
    std::vector<int> map(mapData.data.begin(), mapData.data.end());
    
    /*-------  Find the obstacles & targets ------*/
    // Reserve max sizes for vectors to prevent relocation
    obstacles.reserve(map_height * map_width);
    targets.reserve(map_height * map_width);

    // Traverse map row, column wise while checking each pixel for obstacles and free regions
    for (int i = 1; i < (map_height - 1); i++)
    {
        for (int j = 1; j < (map_width - 1); j++)
        {
            if (map[i*map_width + j] == MAP_PIXEL_OCCUPIED)
            {
                obstacles.emplace_back(j,i);
            }
            else if (map[i*map_width + j] == MAP_PIXEL_UNKNOWN)
            {
                // Check if there are free neighbouring pixels around the unknown pixel
                if (    (map[ i*map_width + (j+1)] == MAP_PIXEL_FREE) ||
                        (map[ i*map_width + (j-1)] == MAP_PIXEL_FREE) ||
                        (map[ (i+1)*map_width + (j)] == MAP_PIXEL_FREE) ||
                        (map[ (i+1)*map_width + (j+1)] == MAP_PIXEL_FREE) ||
                        (map[ (i+1)*map_width + (j-1)] == MAP_PIXEL_FREE) ||
                        (map[ (i-1)*map_width + (j)] == MAP_PIXEL_FREE) ||
                        (map[ (i-1)*map_width + (j+1)] == MAP_PIXEL_FREE) ||
                        (map[ (i-1)*map_width + (j-1)] == MAP_PIXEL_FREE))
                {
                    // Potential target found at (j,i) pixel. 
                    /*------- Check if target has lower cost in costmap  ------*/
                    float loc_x = j * mapData.info.resolution + mapData.info.origin.position.x;
                    float loc_y = i * mapData.info.resolution + mapData.info.origin.position.y;
                    int costmap_pixel_index = ((loc_y - costmapData.info.origin.position.y) / costmapData.info.resolution) * costmapData.info.width +
                                                ((loc_x - costmapData.info.origin.position.x) / costmapData.info.resolution);
                    if (costmapData.data[costmap_pixel_index] > 0) continue;    // (j,i) pixel has high cost. End current iteration

                    /*------- Search for obstacles within inflation radius of the (j,i) pixel  ------*/
                    auto obstacle = obstacles.begin();
                    for (; obstacle != obstacles.end(); obstacle++)
                    {
                        if (abs(obstacle->x - j) + abs(obstacle->y - i) < inflation_radius) break;
                    }
                    if (obstacle == obstacles.end())
                    {
                        // Search completed without any obstacle. Add (j,i) to targets
                        targets.emplace_back(j,i);
                    }
                }
            }
        }
    }
    // Adjust vector sizes (reserve and shrink used for performance optimization)
    obstacles.shrink_to_fit();
    targets.shrink_to_fit();
    return;
}

/**
 * @brief For a given OccupancyGrid, creates a potential map (distance map) taking SourcePoint as starting location (distance = 0)
 * 
 * @param mapData           OccupancyGrid map data
 * @param source_point      Starting point (Zero potential point)
 * @param potential_map
 * @param potential_step    (optional) Potential / Distance increase between two adjacent points in the map. (default 3)      
 * @return true     if potential map creation is successfull
 * @return false    if potential map creation failed
 */
bool MWFCN_Algo::create_potential_map(nav_msgs::msg::OccupancyGrid mapData, Pixel source_point, std::vector<int> &potential_map, int potential_step)
{
    /*------- Initialize potential map with maximum potential ------*/
    potential_map.resize(mapData.data.size());
    potential_map.assign(potential_map.size(), LARGEST_MAP_DISTANCE);

    /*------- Initialize map and current processing points ------*/
    std::vector<int> map(mapData.data.begin(), mapData.data.end());
    int width = mapData.info.width;
    std::vector<Pixel> process_queue;
    process_queue.emplace_back(source_point);
    potential_map[source_point.y * width + source_point.x] = 0;

    int iteration = 1;
    while (process_queue.size() > 0)
    {
        /*
            For all pixels in process_queue, check if adjacent pixels are free.
            Add each adjacent free pixel to adjacent_pixels. Update potentials for added pixels in potential map.
            When adding pixels to adjacent_pixels, set pixel to occupied, to prevent re-processing

            After processing all points in process_queue, replace process_queue with points in adjacent_pixels and repeat process
        */
        std::vector<Pixel> adjacent_pixels;
        for (auto &point : process_queue)
        {
            // Check right pixel
            process_pixel_potential(point, Pixel(point.x + 1, point.y), map, potential_map, width, adjacent_pixels, potential_step);
            // Check left pixel
            process_pixel_potential(point, Pixel(point.x - 1, point.y), map, potential_map, width, adjacent_pixels, potential_step);
            // Check up pixel
            process_pixel_potential(point, Pixel(point.x, point.y + 1), map, potential_map, width, adjacent_pixels, potential_step);
            // Check down pixel
            process_pixel_potential(point, Pixel(point.x, point.y - 1), map, potential_map, width, adjacent_pixels, potential_step);
            // Check upper right pixel
            process_pixel_potential(point, Pixel(point.x + 1, point.y + 1), map, potential_map, width, adjacent_pixels, potential_step);
            // Check upper left pixel
            process_pixel_potential(point, Pixel(point.x - 1, point.y + 1), map, potential_map, width, adjacent_pixels, potential_step);
            // Check lower right pixel
            process_pixel_potential(point, Pixel(point.x + 1, point.y - 1), map, potential_map, width, adjacent_pixels, potential_step);
            // Check lower left pixel
            process_pixel_potential(point, Pixel(point.x - 1, point.y - 1), map, potential_map, width, adjacent_pixels, potential_step);
        }
        process_queue = adjacent_pixels;
        iteration++;

        // Costmap expansion maximum iteration limit check  ( TODO : Check if this is necessary)
        if (iteration > LARGEST_MAP_DISTANCE)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Distance exceeds MAXIMUM SETUP");
            return false;
        }
    }
    return true;
}

/**
 * @brief Given a source pixel and a target pixel, checks if the target pixel is free, calculates its cost and adds to discovered pixels
 * 
 * @param source_pixel          A pixel with a known cost
 * @param target_pixel          An adjacent pixel to source pixel
 * @param map                   A vector containing map data
 * @param potential_map         A vector containing costs for pixels (including source pixel cost)
 * @param map_width             Width of map or potential_map
 * @param discovered_pixels     A vector of free pixels around source_pixel
 * @param unit_potential        Potential increase between adjacent pixels
 */
inline void MWFCN_Algo::process_pixel_potential(Pixel source_pixel, 
                                            Pixel target_pixel, 
                                            std::vector<int> &map, 
                                            std::vector<int> &potential_map, 
                                            int map_width, 
                                            std::vector<Pixel> &discovered_pixels,
                                            int unit_potential)
{
    // Check map bounds
    if ( ((target_pixel.x + target_pixel.y * map_width) < 0) || ((target_pixel.x + target_pixel.y * map_width) >= map.size()) )
    {
        return;
    }

    if (map[target_pixel.x + target_pixel.y * map_width] == MAP_PIXEL_FREE) 
    {
        discovered_pixels.emplace_back(target_pixel.x, target_pixel.y);
        potential_map[target_pixel.x + target_pixel.y * map_width] = potential_map[source_pixel.x + source_pixel.y * map_width] + unit_potential;
        map[target_pixel.x + target_pixel.y * map_width] = MAP_PIXEL_OCCUPIED;
    }
    return;
}


bool MWFCN::map_data_available(){
    // TODO Testing only
    // if (!(get_map_data().data.size() < 1 || get_costmap_data().data.size()<1)){

        if (!(get_map_data().data.size() < 1)){
        map_frame_ = get_map_data().header.frame_id;
        robot_goal_.pose.header.frame_id = map_frame_;
        robot_goal_.pose.pose.position.z = 0;
        robot_goal_.pose.pose.orientation.z = 1.0;
        return true;
    }

    RCLCPP_WARN_STREAM(this->get_logger(), "map data is not available");
    return false;
}  
 
visualization_msgs::msg::Marker MWFCN::create_visualization_msg(int type){

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.lifetime         = rclcpp::Duration(0,0); // TODO : currently points are stored forever

    if (type == LINE) {
        //------------------------------------- initilize the visualized lines
        marker.type				= marker.LINE_LIST;
        marker.action           = marker.ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x 			= 0.03;
        marker.scale.y			= 0.03;
        marker.color.r			= 1.0;   // 0.0/255.0;
        marker.color.g			= 0.0;   // 0.0/255.0;
        marker.color.b 			= 1.0;   // 236.0/255.0;
        marker.color.a 			= 1.0;
    }

    else if(type == POINTS) {
        //------------------------------------- initilize the visualized points
        marker.type 			= marker.POINTS;
        marker.action           = marker.ADD;
        marker.pose.orientation.w =1.0;
        marker.scale.x 			= 0.3; 
        marker.scale.y			= 0.3; 
        marker.color.r 			= 1.0;   // 255.0/255.0;
        marker.color.g 			= 0.0;   // 0.0/255.0;
        marker.color.b 			= 0.0;   // 0.0/255.0;
        marker.color.a			= 1.0;
    }

    else{
        RCLCPP_ERROR_STREAM(MWFCN_Algo::get_logger(), "Undefined visualization msg type");
    }
    return marker;
 } 

/**
 * @brief 
 * 
 * @return true     If success
 * @return false    If failure
 */
bool MWFCN::get_ros_parameters(void)
{
    this->declare_parameter("map_topic", "map"); 
    this->declare_parameter("costmap_topic", "global_costmap/costmap");
    this->declare_parameter("robot_base_frame", "base_footprint");
    this->declare_parameter("robot_frame_prefix", "robot");
    this->declare_parameter("rate", 1.0);
    this->declare_parameter("obstacle_inflation_radius", 6.0);
    this->declare_parameter("robot_count", 1);
    
    map_topic_ = this->get_parameter("map_topic").get_parameter_value().get<std::string>();
    costmap_topic_ = this->get_parameter("costmap_topic").get_parameter_value().get<std::string>();
    robot_base_frame_ = this->get_parameter("robot_base_frame").get_parameter_value().get<std::string>();
    robot_frame_prefix_ = this->get_parameter("robot_frame_prefix").get_parameter_value().get<std::string>();
    rate_ = this->get_parameter("rate").get_parameter_value().get<float>();
    obstacle_inflation_radius_ = this->get_parameter("obstacle_inflation_radius").get_parameter_value().get<float>();
    robot_count_ = this->get_parameter("robot_count").get_parameter_value().get<int>();


    // Remove any leading slash from robot_base_frame
    if (*robot_base_frame_.cbegin() == '/') robot_base_frame_.erase(0, 1);
    // Create fully qualified robot_base_frame names
    for (int i = 1; i < robot_count_ + 1; i++)
    {
        robot_base_frames_.push_back(robot_frame_prefix_ + std::to_string(i) + "/" + robot_base_frame_);
    }

    // Extract robot id from node namespace
    std::string ns = this->get_namespace();
    RCLCPP_INFO_STREAM(this->get_logger(), "Running exploration in namespace: " << ns);
    try{
        std::string id_string = ns.substr(robot_frame_prefix_.size() + 1);
        robot_id_ = std::stoi(id_string);
        RCLCPP_INFO_STREAM(this->get_logger(), "Robot ID: " << robot_id_);
    }
    catch( ... ){
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot extract robot id from node namespace {" << ns 
                                                    << "} with prefix {" << robot_frame_prefix_ << "}");
        return false;
    }

    #ifdef DEBUG
        RCLCPP_INFO_STREAM(MWFCN::get_logger(), "map topic: " << map_topic_
        <<"\ncostmap_topic: " << costmap_topic_
        <<"\nrobot_base_frame: " << robot_base_frame
        <<"\nrate: " << rate_
        <<"\nobstacle_inflation_radius: " << obstacle_inflation_radius_
        <<"\robot_count: " << robot_count_
        <<"\robot_frame_prefix: " << robot_frame_prefix_
      );
    #endif

    return true;
}