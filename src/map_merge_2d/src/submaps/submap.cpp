#include <map_merge_2d/submaps/submap.hpp>

using namespace map_merge_2d;

SubMap::SubMap(rclcpp::Node *node, std::string map_topic) 
:   available(false),
    name(ros_names::parentNamespace(map_topic)),
    logger_(rclcpp::get_logger("SubMap")),
    known_pose_(false)
{
    if (node == nullptr)
    {
        logger_ = node->get_logger();
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("SubMap"), "Invalid node[null] passed to submap creation. Topic: " << map_topic);
        throw std::invalid_argument("Null pointer passed as node to SubMap");
    }

    // Check for initial_pose parameters
    std::string map_namespace = ros_names::parentNamespace(map_topic);
    
    // leading '/' removed using erase before declaring parameters from node 
    // TODO future fix :  ROS2 Humble declare_parameter without default argument is not supported. 
    //              Hence, to check if user parameter is set, default value is set to a junk (very high)
    //              and checked while reading the parameter.
    node->declare_parameter(ros_names::append(map_namespace, "init_pose_x").erase(0,1), 10e5);
    node->declare_parameter(ros_names::append(map_namespace, "init_pose_y").erase(0,1), 10e5);
    node->declare_parameter(ros_names::append(map_namespace, "init_pose_z").erase(0,1), 10e5);
    node->declare_parameter(ros_names::append(map_namespace, "init_pose_yaw").erase(0,1), 10e5);

    double x, y, z, yaw;
    // leading '/' removed using erase before fetching parameters from node 
    x = node->get_parameter(ros_names::append(map_namespace, "init_pose_x").erase(0,1)).as_double();
    y = node->get_parameter(ros_names::append(map_namespace, "init_pose_y").erase(0,1)).as_double();
    z = node->get_parameter(ros_names::append(map_namespace, "init_pose_z").erase(0,1)).as_double();
    yaw = node->get_parameter(ros_names::append(map_namespace, "init_pose_yaw").erase(0,1)).as_double();
    if ((x > 10e4) || (y > 10e4) || (z > 10e4) || (yaw > 10e4))
    {
        RCLCPP_WARN_STREAM(node->get_logger(), map_topic << " initial pose not set. " 
            << "Did you set " << map_namespace << "/init_pose[x,y,z,yaw] parameters? "
            << "We will try to auto calculate initial pose");
        transform_.setIdentity();
    }
    else
    {
        transform_.setOrigin(tf2::Vector3(x, y, z));
        tf2::Quaternion q;
        q.setEuler(yaw, 0.0, 0.0);
        transform_.setRotation(q);
        known_pose_ = true;
        
        RCLCPP_INFO(node->get_logger(), "%s : map subscribed. Initial pose(xyz yaw) [%.3f, %.3f, %.3f, %.3f]",
                                                map_topic.c_str(), x, y, z, yaw);
    }
    
    subscriber_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, 10, 
                                        std::bind(&SubMap::update_map, this, std::placeholders::_1));

}

SubMap::Map SubMap::get_map(void)
{
    std::shared_lock lock(mutex_);
    Map map;
    map.map = map_;
    map.known_pose = known_pose_;
    map.transform_ = transform_;
    map.transform_confidence_ = transform_confidence_;
    map.name_ = name;
    
    return map;
}

void SubMap::update_transform(tf2::Transform transform)
{
    std::unique_lock lock(mutex_);
    transform_ = transform;

    // Set to known_pose configuration once map transform is initialized
    if(!known_pose_)
    {
        RCLCPP_DEBUG(logger_, "%s map transformation established!", name.c_str());
    }
    known_pose_ = true;
}

void SubMap::update_transform(tf2::Transform transform, double confidence)
{
    std::unique_lock lock(mutex_);
    transform_ = transform;
    transform_confidence_ = confidence;

    // Set to known_pose configuration once map transform is initialized
    if(!known_pose_)
    {
        RCLCPP_DEBUG(logger_, "%s map transformation established!", name.c_str());
    }
    known_pose_ = true;
}

void SubMap::update_map(nav_msgs::msg::OccupancyGrid msg)
{
    std::unique_lock lock(mutex_);
    map_ = msg;

    // Set map available flag
    if (!available)
        available = true;
}