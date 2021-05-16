#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;


class PointCloudPublisher : public rclcpp::Node
{
  public:
    PointCloudPublisher()
    : Node("pointcloud_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2::Ptr>("topic", 10);
      timer_ = this->create_wall_timer(
          500ms, std::bind(&PointCloudPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = sensor_msgs::msg::PointCloud2();
      RCLCPP_INFO(this->get_logger(), "Publishing: Pointcloud");
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2::Ptr>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}

