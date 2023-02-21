#ifndef SUBMAP_MATCHER_H
#define SUBMAP_MATCHER_H

#include <functional>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <map_merge_2d/submaps/submap.hpp>
#include <map_merge_2d/cv_core/cv_core.hpp>
#include <map_merge_2d/util/tf_utils.hpp>

namespace map_merge_2d
{
    class SubMapMatcher
    {
        public:
            struct MatcherOptions
            {
                double confidence = 0.5;
            };

            SubMapMatcher(MatcherOptions options);
            SubMapMatcher(rclcpp::Node *node, std::function<std::vector<std::shared_ptr<SubMap>> ()> submap_reader);

            void match(std::vector<std::shared_ptr<SubMap>> submaps);
            void match(void);

        private:
            bool has_known_tf(std::vector<SubMap::Map> maps);
            bool has_known_tf(std::vector<SubMap::Map> maps, std::map<int, cv::Mat> estimates, std::vector<int> &idx);

            rclcpp::Node *node_;
            rclcpp::Logger logger_;
            MatcherOptions options_;
            rclcpp::TimerBase::SharedPtr matcher_timer_;
            std::function<std::vector<std::shared_ptr<SubMap>> ()> submap_reader_;
    };
}  // namespace map_merge_2d

#endif  // SUBMAP_MATCHER_H