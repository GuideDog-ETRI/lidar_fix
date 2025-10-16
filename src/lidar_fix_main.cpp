#include "lidar_fix/lidar_fix_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFix>());
  rclcpp::shutdown();
  return 0;
}
