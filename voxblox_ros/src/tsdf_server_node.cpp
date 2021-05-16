#include <voxblox_ros/tsdf_server.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TsdfServer>());
  rclcpp::shutdown();
  return 0;
}