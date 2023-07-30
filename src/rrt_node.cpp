#include <rrt_explore/rrt.hpp>

using namespace exploration;
using namespace std;

int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RRT>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}