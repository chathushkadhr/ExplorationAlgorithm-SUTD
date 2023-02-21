#ifndef TOPIC_DISCOVERY_H
#define TOPIC_DISCOVERY_H

#include <string>
#include <vector>
#include <functional>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>

#include <map_merge_2d/util/topic_name_utils.hpp>

namespace map_merge_2d
{
    class TopicDiscovery
    {   
        public:
            /**
             * @brief Information of topic to be discovered
             * @example /ground_robots/robot1/map
             * 
             * @param topic_name : map
             * @param topic_namespace : ground_robots (any regex match)
             */
            struct TopicInfo
            {
                std::string topic_name;
                std::string topic_namespace;
                std::string topic_type = "nav_msgs/msg/OccupancyGrid";
                std::vector<std::string> exclusions;
                double discovery_rate = 1.0;
            };

            TopicDiscovery(rclcpp::Node *node, TopicInfo info, std::function<void (std::string)> callback);

        private:
            void discovery_callback();
            std::vector<std::string> get_topic_matches();

            rclcpp::Node *node_;
            TopicInfo info_;
            rclcpp::TimerBase::SharedPtr discover_timer_;
            std::function<void (std::string)> callback_;
            std::unordered_map<std::string, std::string> discovered_topics_; // map {topic_full_name : namespace}
};

}  // namespace map_merge_2d

#endif  // TOPIC_DISCOVERY_H