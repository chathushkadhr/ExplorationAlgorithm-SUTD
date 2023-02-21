#ifndef TF_UTIL_H
#define TF_UTIL_H

#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>

namespace map_merge_2d
{
    namespace tf_utils
    {
        cv::Mat tf2_to_cv_transform(double resolution, tf2::Transform transform);
        tf2::Transform cv_transform_to_tf2(double resolution, cv::Mat transform); 
        tf2::Transform pose_to_tf2(geometry_msgs::msg::Pose pose);

        /**
         * @brief Given a transform for an map image in some reference frame (child frame = image upper left corner)
         *          returns the transform of map origin in the given reference frame
         * 
         * @param map_info 
         * @param transform 
         * @return tf2::Transform 
         */
        tf2::Transform image_to_map_transform(nav_msgs::msg::MapMetaData map_info, cv::Mat transform);

        /**
         * @brief Given a transform for an map image "origin" in some reference frame (child frame = map origin)
         *          returns the transform of image left corner in the given reference frame
         * 
         * @param map_info 
         * @param transform 
         * @return cv::Mat 
         */
        cv::Mat map_to_image_transform(nav_msgs::msg::MapMetaData map_info, tf2::Transform transform);
        
        /**
         * @brief Get the map origin tf : Transform from Image upper left corner to map frame origin
         * 
         * @param map_info 
         * @param transform 
         * @return tf2::Transform 
         */
        tf2::Transform get_map_origin_tf(nav_msgs::msg::MapMetaData map_info);
    }
}

#endif // TF_UTIL_H
