#include <map_merge_2d/map_merger.hpp>

using namespace map_merge_2d;

MapMerger::MapMerger() : Node("map_merge_2d")
{
  // Declare parameters
  this->declare_parameter("discovery_rate", 0.5);
  this->declare_parameter("map_topic", "map");
  this->declare_parameter("map_namespace", "");
  this->declare_parameter("merged_map_topic", "map");

  TopicDiscovery::TopicInfo info;
  info.topic_name = this->get_parameter("map_topic").as_string();
  info.topic_namespace = this->get_parameter("map_namespace").as_string();
  info.exclusions.emplace_back(ros_names::append(this->get_name(), 
                                                this->get_parameter("merged_map_topic").as_string()));
  info.exclusions.emplace_back("/map_merge/map"); // TODO remove. Testing only
  info.topic_type = "nav_msgs/msg/OccupancyGrid";
  info.discovery_rate = this->get_parameter("discovery_rate").as_double();

  // start topic discovery
  map_discovery_ = std::make_shared<TopicDiscovery>(this, info, 
                            std::bind(&MapMerger::topic_discovery_callback, this, std::placeholders::_1));

  // start map matcher
  map_matcher_ = std::make_shared<SubMapMatcher>(this, std::bind(&MapMerger::get_submaps, this));
  
  // start map merger
  map_merger_ = std::make_shared<SubMapMerger>(this, std::bind(&MapMerger::get_submaps, this));

}

void MapMerger::topic_discovery_callback(std::string topic_name)
{
  std::unique_lock lock(submap_mutex_);
  submaps_.emplace_back(std::make_shared<SubMap>(this, topic_name));
}

std::vector<std::shared_ptr<SubMap>> MapMerger::get_submaps()
{
  std::shared_lock lock(submap_mutex_);

  // Return only valid maps (received maps)
  std::vector<std::shared_ptr<SubMap>> submaps;
  for (auto &submap : submaps_)
  {
    if(submap->available)
      submaps.emplace_back(submap);
  }

  return submaps;
}
