#ifndef SUBMAP_H
#define SUBMAP_H

#include <string>
#include <stdexcept>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <map_merge_2d/util/topic_name_utils.hpp>

namespace map_merge_2d
{
    class SubMap
    {
        public:
            struct Map
            {
                nav_msgs::msg::OccupancyGrid map;
                bool known_pose;
                tf2::Transform transform_;
                double transform_confidence_;
                std::string name_;
            };

            SubMap(rclcpp::Node *node, std::string map_topic);

            Map get_map(void);
            void update_transform(tf2::Transform transform);
            void update_transform(tf2::Transform transform, double confidence);

            std::atomic<bool> available;
            const std::string name;

        private:
            void update_map(nav_msgs::msg::OccupancyGrid msg);

            rclcpp::Logger logger_;
            bool known_pose_;
            tf2::Transform transform_;
            double transform_confidence_ = -1;
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscriber_;
            mutable std::shared_mutex mutex_;
            nav_msgs::msg::OccupancyGrid map_;

};

}  // namespace map_merge_2d

#endif  // SUBMAP_H