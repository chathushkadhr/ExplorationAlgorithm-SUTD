#include <map_merge_2d/submaps/submap_merger.hpp>

using namespace map_merge_2d;

SubMapMerger::SubMapMerger()
: logger_(rclcpp::get_logger("SubMapMerger"))
{

}

SubMapMerger::SubMapMerger(rclcpp::Node *node, std::function<std::vector<std::shared_ptr<SubMap>> ()> submap_reader)
: logger_(rclcpp::get_logger("SubMapMerger"))
{
    if (node == nullptr)
    {
        RCLCPP_ERROR_STREAM(logger_, "Invalid node[null] passed to submap merger.");
        return;
    }

    if (submap_reader == nullptr)
    {
        RCLCPP_ERROR_STREAM(logger_, "Invalid submap_reader[null] passed to submap merger.");
        return;
    }

    node_ = node;
    logger_ = node_->get_logger();
    submap_reader_ = submap_reader;

    // Declare parameters
    node_->declare_parameter("merging_rate", 0.3);
    node_->declare_parameter("publish_merged_map", true);
    node_->declare_parameter("publish_tf", true);
    node_->declare_parameter("world_frame", "world");

    // Get Parameters
    double merging_rate = node_->get_parameter("merging_rate").as_double();
    publish_merged_map_ = node_->get_parameter("publish_merged_map").as_bool();
    publish_tf = node_->get_parameter("publish_tf").as_bool();
    std::string merged_map_topic = node_->get_parameter("merged_map_topic").as_string();
    world_frame_ = node_->get_parameter("world_frame").as_string();

    // Create tf broadcaster
    tf_broadcaster_ =  std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    // Create publisher
    map_publisher_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(merged_map_topic, 10);

    // Create timers
    merging_timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0/merging_rate),
                                std::bind((void(SubMapMerger::*)(void))&SubMapMerger::merge, this));
}

void SubMapMerger::merge(void)
{
    merge(submap_reader_());
}

bool SubMapMerger::merge(std::vector<std::shared_ptr<SubMap>> submaps)
{   
    
    // Submap vector should at least have one map
    if (submaps.size() < 1)
    {
        RCLCPP_WARN(logger_, "No submaps received!");
        return false;
    }

    // Read available maps with known transforms
    std::vector<SubMap::Map> maps;
    for (auto &submap : submaps)
    { 
        if (submap->available)
        {
            SubMap::Map _map_ = submap->get_map();
            if (_map_.known_pose)
                maps.emplace_back(_map_);
        }
    }

    // If no map has transforms, end merge
    if (!maps.size())
    {
        RCLCPP_WARN(logger_, "Submaps not available for merging");
        return false;
    }

    // Convert OccupancyGrids to CV Mat and transform the maps
    std::vector<cv_core::CVImage> cv_maps;
    for (auto &map : maps)
    {
        cv::Mat cv_map = cv::Mat(map.map.info.height,
                                    map.map.info.width,
                                    CV_8UC1,
                                    map.map.data.data());
        
        // Check if converted maps are valid
        if (! (cv_map.rows > 0 && cv_map.cols > 0))
        {
            return false;
        }

        cv_core::CVImage transformed_map;
        /* The map transform is inverted, because for merging, we need to remove the existing frame shift */
        // cv::Mat cv_transform = tf_utils::map_to_image_transform(map.map.info, map.transform_.inverse());
        cv::Mat cv_transform = tf_utils::tf2_to_cv_transform(
                                            map.map.info.resolution,
                                            map.transform_ * tf_utils::get_map_origin_tf(map.map.info).inverse());

        cv_core::image_transform(cv_core::CVImage(cv_map), transformed_map, cv_transform);
        cv_maps.emplace_back(transformed_map);
    }

    // Create merged canvas
    cv_core::CVImage merged_image = merge_map_images(cv_maps);

    // Publish tf
    if (publish_tf)
    {
        publish_map_transforms(maps);
    }

    // Currently supports same resolution maps only.
    /* TODO support multiresolution maps */
    if (publish_merged_map_)
    {
        publish_map(merged_image, maps.front().map.info.resolution);
    }
    

    // // tf printer
    // tf2::Transform mytf = transform;
    // tf2::Matrix3x3 myrot = mytf.getBasis();
    // tf2::Vector3 mytrans = mytf.getOrigin();
    // std::cout << "[" << myrot[0][0] << "," << myrot[0][1] << "," << myrot[0][2] << "\n " <<
    //                     myrot[1][0] << "," << myrot[1][1] << "," << myrot[1][2] << "\n " <<
    //                     myrot[2][0] << "," << myrot[2][1] << "," << myrot[2][2] << "]" << std::endl;

    // std::cout << "[" << mytrans[0] << "," << mytrans[1] << "," << mytrans[2] << "]" << std::endl;
    // std::cout << cv_transform << std::endl;
    return true;
}

cv_core::CVImage SubMapMerger::merge_map_images(std::vector<cv_core::CVImage> images)
{
    double min_x = images.front().origin.x;
    double min_y = images.front().origin.y;
    double max_x = images.front().origin.x + images.front().image.cols;
    double max_y = images.front().origin.y + images.front().image.rows;
    for (auto &image : images)
    {
        /* Find minimum map image coordinates [x,y] */
        if (image.origin.x < min_x) min_x = image.origin.x;
        if (image.origin.y < min_y) min_y = image.origin.y;

        /* Find maximum image coordinates */
        if ( (image.origin.x + image.image.cols) > max_x ) max_x = image.origin.x + image.image.cols;
        if ( (image.origin.y + image.image.rows) > max_y ) max_y = image.origin.y + image.image.rows;
    }

    /* Transform images */
    std::vector<cv::Mat> transformed_images;
    cv::Size dsize(max_x - min_x, max_y - min_y);

    for (auto &image : images)
    {
        /* Translate all images by minimum offsets + individual image offset */
        /* This will align all maps to optimal positive pixel coefficient area */
        double translation_mat[] = {1, 0, (-min_x + image.origin.x), 
                                    0, 1, (-min_y + image.origin.y)};
        cv::Mat transform = cv::Mat(2, 3, CV_64FC1, translation_mat);

        cv::Mat dest;
        cv::warpAffine(image.image, dest, transform, dsize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255)); // 255: White border
        transformed_images.emplace_back(dest);
    }
    
    /* Merge all images into one canvas */
    cv_core::CVImage merged_image(cv::Mat(dsize.height, dsize.width, CV_8UC1, 255), min_x, min_y);
    cv::Mat merged_map_obstacles = merged_image.image.clone();
    merged_map_obstacles.setTo(0, merged_image.image > 100); // Clear unknown area
    for (auto &image : transformed_images)
    {
        /**
         * @brief Copy obstacles into merged image (accumulated)
         * 
         * Select obstacle pixels from merged map and submap
         * Add pixel probabilities and clamp to 100
         */
        cv::Mat sub_map_obstacles = image.clone();
        sub_map_obstacles.setTo(0, image > 100); // Clear unknown area
        merged_map_obstacles += sub_map_obstacles;
        merged_map_obstacles.setTo(100, merged_map_obstacles > 100); // Clamp probabilities to 100
        /**
         * @brief Copy known free area to merged map
         * 
         * if a pixel in merged map is unknown -> 255
         * and
         * if corresponding pixel in a sub map is known-free -> 0
         * Then mark merged map pixel as known-free.
         * (If any submap marks this pixel as occupied, that will take precidence)
         */
        merged_image.image.setTo(0,  image == 0); //merged_image.image == 255 &
    }
    merged_map_obstacles.copyTo(merged_image.image, merged_map_obstacles > 0); // Copy obstacle values to merged map

    for (auto &image : transformed_images)
    {
        /**
         * @brief Copy  all known free areas to merged map
         * 
         * if corresponding pixel in a sub map is known-free -> 0
         * Then mark merged map pixel as known-free.
         */
        merged_image.image.setTo(0,  image == 0); 
    }

    // cv::Mat scaled_image;
    // cv::Size scale(dsize.width * 3, dsize.height * 3);
    // resize(merged_image.image, scaled_image, scale, cv::INTER_LINEAR);
    // scaled_image = (255 - scaled_image); // Inverting colors for visualization
    // scaled_image.setTo(150, scaled_image == 0); // Changing unknown area color
    // cv::imshow("merged", scaled_image);
    // cv::waitKey(500);

    return merged_image;
}

void SubMapMerger::publish_map(cv_core::CVImage map, double resolution)
{
    nav_msgs::msg::OccupancyGrid map_msg;
    map_msg.header.frame_id = world_frame_;
    map_msg.header.stamp = node_->get_clock()->now();
    map_msg.info.height = map.image.rows;
    map_msg.info.width = map.image.cols;
    map_msg.info.resolution = resolution;
    map_msg.info.origin.position.x = map.origin.x * resolution;
    map_msg.info.origin.position.y = map.origin.y * resolution;
    map_msg.data.assign(map.image.begin<int8_t>(), map.image.end<int8_t>());
    map_publisher_->publish(map_msg);
}

void SubMapMerger::publish_map_transforms(std::vector<SubMap::Map> maps)
{
    for (auto &map : maps)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = world_frame_;
        transform.header.stamp = node_->get_clock()->now();
        transform.child_frame_id = map.name_ + "/map";

        tf2::Vector3 origin = map.transform_.getOrigin();
        tf2::Quaternion rotation = map.transform_.getRotation();
        transform.transform.translation.x = origin.getX();
        transform.transform.translation.y = origin.getY();
        transform.transform.translation.z = origin.getZ();

        transform.transform.rotation.x = rotation.getX();
        transform.transform.rotation.y = rotation.getY();
        transform.transform.rotation.z = rotation.getZ();
        transform.transform.rotation.w = rotation.getW();

        tf_broadcaster_->sendTransform(transform);
    }
}