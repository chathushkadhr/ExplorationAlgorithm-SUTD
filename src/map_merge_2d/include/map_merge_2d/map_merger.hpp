#ifndef MAP_MERGE_2D_H
#define MAP_MERGE_2D_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <mutex>
#include <shared_mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <map_merge_2d/submaps/submap.hpp>
#include <map_merge_2d/util/topic_name_utils.hpp>
#include <map_merge_2d/map_discovery/topic_discovery.hpp>
#include <map_merge_2d/submaps/submap_matcher.hpp>
#include <map_merge_2d/submaps/submap_merger.hpp>

namespace map_merge_2d
{
    class MapMerger : public rclcpp::Node
    {
        public:
            MapMerger();

        private:
            void topic_discovery_callback(std::string topic_name);
            std::vector<std::shared_ptr<SubMap>> get_submaps();
            void publish_tf(void);

            std::shared_ptr<TopicDiscovery> map_discovery_;
            std::shared_ptr<SubMapMatcher> map_matcher_;
            std::shared_ptr<SubMapMerger> map_merger_;

            // Protected variables - to be accessed with thread safety only
            std::vector<std::shared_ptr<SubMap>> submaps_;
            mutable std::shared_mutex submap_mutex_;
};

}  // namespace map_merge_2d

#endif  // MAP_MERGE_2D_H