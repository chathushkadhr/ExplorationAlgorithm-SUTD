#ifndef SUBMAP_MERGER_H
#define SUBMAP_MERGER_H

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <map_merge_2d/submaps/submap.hpp>
#include <map_merge_2d/cv_core/cv_core.hpp>
#include <map_merge_2d/util/tf_utils.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/transform_datatypes.h>

namespace map_merge_2d
{
    class SubMapMerger
    {
        public:
            SubMapMerger();
            SubMapMerger(rclcpp::Node *node, std::function<std::vector<std::shared_ptr<SubMap>> ()> submap_reader);

            void merge(void);
            bool merge(std::vector<std::shared_ptr<SubMap>> submaps);
        
        private:
            cv_core::CVImage merge_map_images(std::vector<cv_core::CVImage> images);
            void publish_map(cv_core::CVImage map, double resolution);
            void publish_map_transforms(std::vector<SubMap::Map> maps);

            rclcpp::Node *node_;
            rclcpp::Logger logger_;
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
            std::function<std::vector<std::shared_ptr<SubMap>> ()> submap_reader_;
            rclcpp::TimerBase::SharedPtr merging_timer_;
            bool publish_merged_map_ = false;
            bool publish_tf = false;
            std::string world_frame_;
            
    }; // namespace submap_merger

} // namespace map_merge_2d



# endif // SUBMAP_MERGER_H