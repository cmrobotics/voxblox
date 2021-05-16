//#include <chrono>
//#include <functional>
//#include <memory>
//#include <queue>
//#include <string>

#include <rclcpp/rclcpp.hpp>
//#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <voxblox/core/common.h>
#include <voxblox/utils/color_maps.h>
//#include <voxblox_ros/transformer.h>
//#include <std_msgs/msg/string.hpp>

//using std::placeholders::_1;

//using namespace std::chrono_literals;


class TsdfServer : public rclcpp::Node
{
  private:
    /// Colormap to use for intensity pointclouds.
    std::shared_ptr<voxblox::ColorMap> color_map_;

  public:
    TsdfServer(): Node("tsdf_server") {
      //pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      //  "pointcloud", 10, std::bind(&TsdfServer::pointcloud_sub_callback, this, _1));
    }
    void processPointCloudMessageAndInsert(
      const sensor_msgs::msg::PointCloud2::Ptr& pointcloud_msg,
      const voxblox::Transformation& T_G_C, const bool is_freespace_pointcloud);

};
