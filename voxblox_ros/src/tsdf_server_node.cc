#include "voxblox_ros/tsdf_server.h"

#include <gflags/gflags.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("voxblox");

  voxblox::TsdfServer node(nh, nh_private);
  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok())
  {
    RCLCPP_INFO(node->get_logger(), "Alive\n");
    chatter_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  return 0;
}
