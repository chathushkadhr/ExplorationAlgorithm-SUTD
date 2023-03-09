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
    target_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontier_targets", 10);

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
    if (!get_transform(map_frame_, robot_base_frame_, map_to_baseframe)) return;

    /*-------  Fetch external data------*/
    nav_msgs::msg::OccupancyGrid mapData = get_map_data();  
    nav_msgs::msg::OccupancyGrid costmapData = get_costmap_data(); 

    /*------- Extract obstacles and exploration targets from maps ------*/
    std::vector<Pixel> obstacles;
    std::vector<Pixel> targets;
    process_maps(mapData, costmapData, obstacles, targets);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Targets count: " << std::to_string(targets.size()));

    /*-------Check exploration stop condition ------*/
    if (targets.size() == 0)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Exploration done!");
        return;
    }

    /*------- Cluster targets into different groups and find the center of each group   ------*/
    std::vector<Cluster> target_clusters = cluster_2D(targets);

    /*------- Receive robots' locations ------*/
    std::vector<geometry_msgs::msg::TransformStamped> robot_transforms;
    robot_transforms.push_back(map_to_baseframe); // Insert current robot's location
    for (int i = 1; i < robot_count_; i++)
    {
        geometry_msgs::msg::TransformStamped transform;
        if (get_transform(map_frame_, robot_base_frames_[i], transform))
        {
            robot_transforms.push_back(transform);
        }
    }

    /*------- Initialize the distance map (potential map) ------*/
    std::vector<std::vector<int>> robot_potential_maps;
    robot_potential_maps.reserve(robot_transforms.size());
    for (auto &robot_transform : robot_transforms)
    {
        std::vector<int> distance_map;
        int robot_location_x = (robot_transform.transform.translation.x - mapData.info.origin.position.x) / mapData.info.resolution;
        int robot_location_y = (robot_transform.transform.translation.y - mapData.info.origin.position.y) / mapData.info.resolution;
        create_potential_map(mapData, Pixel(robot_location_x, robot_location_y), distance_map);
        robot_potential_maps.push_back(distance_map);
    }

    /*------- Calculate attraction to each target cluster ------*/
    Pixel best_cluster(target_clusters.back().center());
    float max_attraction = calculate_attraction(robot_potential_maps, mapData.info.width, target_clusters.back());
    target_clusters.pop_back();
    for (auto &cluster : target_clusters)
    {
        float attraction = calculate_attraction(robot_potential_maps, mapData.info.width, cluster);
        if (attraction > max_attraction)
        {
            best_cluster = cluster.center();
            max_attraction = attraction;
        }
    }

    // Display clusters and attraction
    visualization_msgs::msg::MarkerArray target_cluster_markers;
    int id = 0;
    for (auto &cluster : target_clusters)
    {
        // Cluster visualization
        target_cluster_markers.markers.push_back(create_visualization_msg(POINTS));
        target_cluster_markers.markers.back().id = id++;
        target_cluster_markers.markers.back().points.push_back(geometry_msgs::msg::Point().set__x(
            cluster.x * mapData.info.resolution + mapData.info.origin.position.x).set__y(
            cluster.y * mapData.info.resolution + mapData.info.origin.position.y));

        float attraction = calculate_attraction(robot_potential_maps, mapData.info.width, cluster);
        target_cluster_markers.markers.back().scale.x = attraction / max_attraction;
        target_cluster_markers.markers.back().scale.y = attraction / max_attraction;
        target_cluster_markers.markers.back().scale.z = attraction / max_attraction;

        // Highlight best target in green
        if (attraction == max_attraction)
        {
            target_cluster_markers.markers.back().color.r = 0.0;
            target_cluster_markers.markers.back().color.g = 1.0;
            target_cluster_markers.markers.back().color.b = 0.0;
        }
    }
    target_publisher_->publish(target_cluster_markers);

    /*------- Send cluster location with max attraction as goal ------*/
    robot_goal_.pose.header.stamp = rclcpp::Time(0);
    robot_goal_.pose.pose.position.x = best_cluster.x * mapData.info.resolution + mapData.info.origin.position.x;
    robot_goal_.pose.pose.position.y = best_cluster.y * mapData.info.resolution + mapData.info.origin.position.y;
    RCLCPP_INFO_STREAM(this->get_logger(), "Goal: " << robot_goal_.pose.pose.position.x << ", " << robot_goal_.pose.pose.position.y);
    this->navigation_client_->async_send_goal(robot_goal_);
}

void MWFCN::map_callback(const nav_msgs::msg::OccupancyGrid msg)
{
    set_map_data(msg);
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
 * @return std::vector<MWFCN::Cluster> A vector of Clusters : [cluster center(x,y), number of points within the cluster]
 */
std::vector<MWFCN::Cluster> MWFCN::cluster_2D(std::vector<MWFCN::Pixel> points, int proximity_threshold)
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
void MWFCN::process_maps(nav_msgs::msg::OccupancyGrid mapData, nav_msgs::msg::OccupancyGrid costmapData, 
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
                    (void)costmapData;
                    /* TODO consider about worldmap  / robot frame to costmap frame rotation also. And map bound checking
                    float loc_x = j * mapData.info.resolution + mapData.info.origin.position.x;
                    float loc_y = i * mapData.info.resolution + mapData.info.origin.position.y;
                    int costmap_pixel_index = ((loc_y - costmapData.info.origin.position.y) / costmapData.info.resolution) * costmapData.info.width +
                                                ((loc_x - costmapData.info.origin.position.x) / costmapData.info.resolution);
                    if (costmapData.data[costmap_pixel_index] > 0) continue;    // (j,i) pixel has high cost. End current iteration
                    */

                    /*------- Search for obstacles within inflation radius of the (j,i) pixel  ------*/
                    auto obstacle = obstacles.begin();
                    for (; obstacle != obstacles.end(); obstacle++)
                    {
                        if (abs(obstacle->x - j) + abs(obstacle->y - i) < obstacle_inflation_radius_) break;
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
bool MWFCN::create_potential_map(nav_msgs::msg::OccupancyGrid mapData, Pixel source_point, std::vector<int> &potential_map, int potential_step)
{
    /*------- Initialize potential map with maximum potential ------*/
    potential_map.resize(mapData.data.size());
    potential_map.assign(potential_map.size(), 1000); //LARGEST_MAP_DISTANCE

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
        adjacent_pixels.clear();
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
inline void MWFCN::process_pixel_potential(Pixel source_pixel, 
                                            Pixel target_pixel, 
                                            std::vector<int> &map, 
                                            std::vector<int> &potential_map, 
                                            int map_width, 
                                            std::vector<Pixel> &discovered_pixels,
                                            int unit_potential)
{
    // Check map bounds
    if ( ((target_pixel.x + target_pixel.y * map_width) < 0) || ((target_pixel.x + target_pixel.y * map_width) >= (int)map.size()) )
    {
        return;
    }

    // Potential map is extended to unknown regions also
    if ( (map[target_pixel.x + target_pixel.y * map_width] == MAP_PIXEL_FREE) || (map[target_pixel.x + target_pixel.y * map_width] == MAP_PIXEL_UNKNOWN) )
    {
        discovered_pixels.emplace_back(target_pixel.x, target_pixel.y);
        potential_map[target_pixel.x + target_pixel.y * map_width] = potential_map[source_pixel.x + source_pixel.y * map_width] + unit_potential;
        map[target_pixel.x + target_pixel.y * map_width] = MAP_PIXEL_OCCUPIED;
    }
    return;
}

/**
 * @brief Given a set of robot potential maps and a target, returns attaction of the target to the first robot in the vector
 * 
 * @param robot_potential_maps  Potential maps for each robot. First potential map should belong to the host robot. Attraction value corresponds to this robot only.
 * @param target 
 * @return float 
 */
float MWFCN::calculate_attraction(std::vector<std::vector<int>> robot_potential_maps, int map_width, Cluster target)
{
    // Attraction due to cluster size
    float size_attraction = 1.0 - 0.9 * exp(-target.size / 100.0); // Slow varying function from 0.1 to 1

    // Host robot attraction to target    
    float distance_attraction =  0.1 + 0.9 * exp(-robot_potential_maps[0][target.y * map_width + target.x] / 100.0);     // [ 0.1 to 1]

    // Repulsion from other robots   
    float attraction_factor = 1.0;
    for (uint i = 1; i < robot_potential_maps.size(); i++)
    {   
        //attraction_factor *= ( 1 - exp( -robot_potential_maps[i][target.y * map_width + target.x] / 1000.0 )); // Slow varying function from 0 to 1
        attraction_factor *= 
    }

    float attraction = size_attraction * distance_attraction * attraction_factor;
    return attraction;
}

bool MWFCN::map_data_available(){

    if (!(get_map_data().data.size() < 1 || get_costmap_data().data.size()<1)){
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
    marker.lifetime         = rclcpp::Duration(0,0);

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
        RCLCPP_ERROR_STREAM(MWFCN::get_logger(), "Undefined visualization msg type");
    }
    return marker;
 } 

void MWFCN::get_ros_parameters(void)
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