#include <exploration/exploration_code.hpp>

using namespace exploration;
using namespace std;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MWFCN_Algo>());
  rclcpp::shutdown();
  return 0;
}