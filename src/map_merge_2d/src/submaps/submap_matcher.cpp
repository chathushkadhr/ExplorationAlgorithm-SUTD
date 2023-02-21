#include <map_merge_2d/submaps/submap_matcher.hpp>

using namespace map_merge_2d;

SubMapMatcher::SubMapMatcher(MatcherOptions options)
: logger_(rclcpp::get_logger("SubMapMatcher"))
{
    options_ = options;
}

SubMapMatcher::SubMapMatcher(rclcpp::Node *node, std::function<std::vector<std::shared_ptr<SubMap>> ()> submap_reader) 
: logger_(rclcpp::get_logger("SubMapMatcher"))
{
    if (node == nullptr)
    {
        RCLCPP_ERROR_STREAM(logger_, "Invalid node[null] passed to submap matcher.");
        return;
    }

    if (submap_reader == nullptr)
    {
        RCLCPP_ERROR_STREAM(logger_, "Invalid submap_reader[null] passed to submap matcher.");
        return;
    }

    node_ = node;
    logger_ = node_->get_logger();
    submap_reader_ = submap_reader;

    // Declare parameters
    node_->declare_parameter("matching_rate", 0.3);
    node_->declare_parameter("matching_confidence", 0.5);

    // Get Parameters
    double matching_rate = node_->get_parameter("matching_rate").as_double();
    options_.confidence = node_->get_parameter("matching_confidence").as_double();

    // Create timers
    matcher_timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0/matching_rate),
                                std::bind((void(SubMapMatcher::*)(void))&SubMapMatcher::match, this));
}   

void SubMapMatcher::match(void)
{
    match(submap_reader_());
}

void SubMapMatcher::match(std::vector<std::shared_ptr<SubMap>> submaps)
{
    // Submap vector should at least have one map
    if (submaps.size() < 1)
    {
        RCLCPP_WARN(logger_, "No submaps available for matching");
        return;
    }

    // Read maps
    std::vector<SubMap::Map> maps;
    for (auto &submap : submaps)
    { 
        if (!submap->available)
        {
            // Abort matching. Unavailable submaps passed
            return;
        }

        maps.emplace_back(submap->get_map());
    }

    // Convert OccupancyGrids to CV Mat
    std::vector<cv::Mat> cv_maps;
    cv_maps.reserve(submaps.size());
    for (auto &map : maps)
    {
        cv_maps.emplace_back(map.map.info.height, 
                                map.map.info.width, 
                                CV_8UC1, 
                                map.map.data.data());

        if (!(cv_maps.back().cols > 0 && cv_maps.back().rows > 0))
        {
            // Empty submaps passed
            return;
        }

        /**
         *  OccupancyGrid 'unknown' areas are denoted by -1
         *  When converting OccupancyGrid to CV Mat, [signed to unsigned], 
         *  unknown areas will get converted to white regions (255). 
         */

        /**
         *  OccupancyGrid 'known free' areas are denoted by 0
         *  These areas will get converted to black regions (0). 
         */

        /* For feature mapping, we do not consider 'known free' area. Only obstacles */
        cv_maps.back().setTo(255, cv_maps.back() == 0);
    }
    cv_maps.shrink_to_fit();

    /* Estimate map transforms */
    std::map<int, double> transform_confidence;
    std::map<int, cv::Mat> relative_transforms = cv_core::estimateTransforms(cv_maps, 
                                                        cv_core::FeatureType::AKAZE, 
                                                        options_.confidence,
                                                        transform_confidence);

    /* Check if sufficient matches available. Else abort */
    if (relative_transforms.size() < 2)
    {
        RCLCPP_INFO_STREAM_THROTTLE(logger_, *node_->get_clock(), 10000, "Failed to create sufficient matches. Try reducing confidence score?");
        return;
    }

    /* Publish warning for unmatched maps */
    if (submaps.size() > relative_transforms.size())
    {
        std::string unmatched_maps_msg = "Could not estimate transforms for maps: ";
        for (uint i = 0; i < submaps.size(); i++)
        {
            if (!relative_transforms.count(i))
                unmatched_maps_msg += maps.at(i).name_ + " ";
        }
        RCLCPP_DEBUG_STREAM(logger_, unmatched_maps_msg);
    } 

    /* Check if any map has known tf. If not initialize a map with identity transform */
    if (!has_known_tf(maps))
    {
        tf2::Transform initial_tf;
        initial_tf.setIdentity();
        submaps.at(relative_transforms.begin()->first)->update_transform(initial_tf);
        // Update local map transform
        maps.at(relative_transforms.begin()->first).transform_.setIdentity();
        maps.at(relative_transforms.begin()->first).known_pose = true;
    }

    /* Check for matched map with known transform */
    std::vector<int> anchor_maps;
    if (has_known_tf(maps, relative_transforms, anchor_maps))
    {
        /**
         * @note T_a  : anchor map transform in global reference frame
         *       T_cv_a : anchor map transform in CV matching reference frame (relative transform)
         *       T_cv_b : another map transform in CV matching reference frame (relative transform)
         *      
         *       To convert 'b' map to global frame,
         *          T_b = T_a * inverse(T_cv_a) * T_cv_b
         * 
         *       anchor_tf => T_a * inverse(T_cv_a)
         */
        tf2::Transform anchor_tf = maps.at(anchor_maps.front()).transform_ * 
                                    tf_utils::get_map_origin_tf(
                                        maps.at(anchor_maps.front()).map.info).inverse() *
                                    tf_utils::cv_transform_to_tf2(
                                        maps.at(anchor_maps.front()).map.info.resolution,
                                        relative_transforms.at(anchor_maps.front()));

        // Remove anchor map from relative transform list
        relative_transforms.erase(anchor_maps.front());

        for (auto &relative_transform : relative_transforms)
        {
            tf2::Transform submap_transform = anchor_tf * 
                                                tf_utils::cv_transform_to_tf2(
                                                    maps.at(relative_transform.first).map.info.resolution,
                                                    relative_transform.second).inverse() *
                                                tf_utils::get_map_origin_tf(
                                                    maps.at(relative_transform.first).map.info);

            if (transform_confidence.at(relative_transform.first) > maps.at(relative_transform.first).transform_confidence_)
            {
                double x =  submap_transform.getOrigin().getX();
                double y = submap_transform.getOrigin().getY();
                double q_z = submap_transform.getRotation().getZ();

                if (maps.at(relative_transform.first).known_pose)
                {
                    // Add transform validity check using existing transfrom 
                    // (Sudden large deviations in transforms should indicate some error)
                    double prev_x = maps.at(relative_transform.first).transform_.getOrigin().getX();
                    double prev_y = maps.at(relative_transform.first).transform_.getOrigin().getY();
                    double prev_q_z = maps.at(relative_transform.first).transform_.getRotation().getZ();

                    if ( (abs(prev_x - x) + abs(prev_y - y) > 5.0) || (abs(prev_q_z - q_z) > 1.0) )
                    {
                        RCLCPP_WARN_STREAM(logger_, "Large changes in submap transform detected. Did you set a wrong initial transform?" <<
                                                    " Other possible causes : opencv feature matcher divergence" <<
                                                    "\n [x, y, q.z] : " << "[" << prev_x << ", "  << prev_y << ", " << prev_q_z << "]" <<
                                                    " to [" << x << ", "  << y << ", " << q_z << "]");
                        return;
                    }
                }

                RCLCPP_INFO_STREAM(logger_, "Map " << maps.at(relative_transform.first).name_ << "transform updated.\n " <<
                                                        "confidence: " << transform_confidence.at(relative_transform.first) <<
                                                        "\ntransform[x, y, q.z]: [" << x << ", "  << y << ", " << q_z << "]");
                submaps.at(relative_transform.first)->update_transform(submap_transform, 
                                                                        transform_confidence.at(relative_transform.first));
            }
        }
    }
    else
    {
        std::string unlinked_anchor_msg = "Cannot find transform between existing anchor map and maps : ";
        for (auto &itr : relative_transforms)
            unlinked_anchor_msg += maps.at(itr.first).name_ + " ";
        RCLCPP_WARN_STREAM(logger_, unlinked_anchor_msg);
    }
}

/**
 * @brief Check for any map with known transforms
 * 
 * @param maps 
 * @return true 
 * @return false 
 */
bool SubMapMatcher::has_known_tf(std::vector<SubMap::Map> maps)
{
    for (auto &map : maps)
    {
        if (map.known_pose)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Checks for maps with known transforms within the given mask
 * 
 * @param maps 
 * @param mask 
 * @param idx     ids of maps with known_tf within the given mask
 * @return true 
 * @return false 
 */
bool SubMapMatcher::has_known_tf(std::vector<SubMap::Map> maps, std::map<int, cv::Mat> estimates, std::vector<int> &idx)
{
    idx.clear();

    for (auto &estimate : estimates)
    {
        if (maps.at(estimate.first).known_pose)
        {
            idx.emplace_back(estimate.first);
        }
    } 

    if (idx.size())
    {
        return true;
    }
    else
    {
        return false;
    }
    
}