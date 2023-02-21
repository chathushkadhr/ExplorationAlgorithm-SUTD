#include <map_merge_2d/map_discovery/topic_discovery.hpp>

using namespace map_merge_2d;

/**
 * @brief Construct a new Topic Discovery:: Topic Discovery object
 * 
 * @param node 
 * @param info
 * @param callback discovered callback(topic_full_name, topic_namespace)
 */
TopicDiscovery::TopicDiscovery(rclcpp::Node *node, TopicInfo info, std::function<void (std::string)> callback)
{
    if (node == nullptr)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("TopicDiscovery"), "Null node pointer passed for discovery");
        return;
    }

    if (info.topic_name == "")
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("TopicDiscovery"), "Cannot discover an empty topic");
        return;
    }

    if (info.discovery_rate <= 0)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("TopicDiscovery"), "Discovery rate should be positive. Given: " << info.discovery_rate);
        return;
    }

    if (callback == nullptr)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("TopicDiscovery"), "Discovery without a callback initiated");
        return;
    }

    // Store args
    node_ = node;
    info_ = info;
    callback_ = callback;

    // Create timers
    discover_timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0/info_.discovery_rate),
                                                std::bind(&TopicDiscovery::discovery_callback, this));
}

void TopicDiscovery::discovery_callback()
{
  std::vector<std::string> topics = get_topic_matches();
  for (const auto& topic : topics)
  {
    if(!discovered_topics_.count(topic))
    {
        // New topic discovered
        discovered_topics_.insert({topic, ros_names::parentNamespace(topic)});
        callback_(topic);
    }
  }
}

std::vector<std::string> TopicDiscovery::get_topic_matches()
{
    std::vector<std::string> topics;
    
    std::map<std::string, std::vector<std::string>> topic_infos = node_->get_topic_names_and_types();
    for (const auto& topic_it : topic_infos) 
    {
        std::string topic_name = topic_it.first;
        std::vector<std::string> topic_types = topic_it.second;
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("submap_utils"), "topic_name: " << topic_name);

        // Iterate over all topic types
        for (const auto& topic_type : topic_types) 
        {
            // test whether topic matches given topic name 
            std::string topic_namespace = ros_names::parentNamespace(topic_name);
            bool is_map_topic = (ros_names::append(topic_namespace, info_.topic_name) == topic_name);

            // test whether topic contains *anywhere* given topic namespace 
            auto pos = topic_name.find(info_.topic_namespace);
            bool contains_namespace = (pos != std::string::npos);

            // check topic type match
            bool is_type_match = (topic_type == info_.topic_type);

            // check topic exclusions list
            auto itr = std::find(info_.exclusions.begin(), info_.exclusions.end(), topic_name);
            bool excluded_topic = (itr != info_.exclusions.end());

            if (is_map_topic && contains_namespace && is_type_match && (!excluded_topic))
                topics.emplace_back(topic_name);
        }
    }
    return topics;
}

