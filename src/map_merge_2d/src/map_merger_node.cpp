#include <map_merge_2d/map_merger.hpp>

using namespace map_merge_2d;

int main([[maybe_unused]]int argc, [[maybe_unused]]char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMerger>());
  rclcpp::shutdown();
  return 0;
}