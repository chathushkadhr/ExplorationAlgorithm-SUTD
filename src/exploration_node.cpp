#include <exploration/exploration_code.hpp>

using namespace exploration;
using namespace std;

int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MWFCN_Algo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
