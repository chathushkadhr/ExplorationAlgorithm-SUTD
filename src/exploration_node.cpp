#include <exploration/mwfcn.hpp>

using namespace exploration;
using namespace std;

int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MWFCN>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}