//#include <chrono>
//#include <functional>
//#include <memory>
//#include <queue>
//#include <string>

#include <rclcpp/rclcpp.hpp>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/core/tsdf_map.h>
//#include <std_msgs/msg/string.hpp>
#include <voxblox/alignment/icp.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <voxblox/core/common.h>
#include <voxblox/utils/color_maps.h>
#include <tf2_ros/transform_broadcaster.h>

//#include <voxblox_ros/transformer.h>
//#include <std_msgs/msg/string.hpp>

//using std::placeholders::_1;

//using namespace std::chrono_literals;


class TsdfServer : public rclcpp::Node
{
  private:
    /// Colormap to use for intensity pointclouds.
    std::shared_ptr<voxblox::ColorMap> color_map_;

    /// What output information to publish
    bool publish_pointclouds_on_update_;
    bool publish_slices_;
    bool publish_pointclouds_;
    bool publish_tsdf_map_;

    /// Whether to save the latest mesh message sent (for inheriting classes).
    bool cache_mesh_;

    /**
     *Whether to enable ICP corrections. Every pointcloud coming in will attempt
    * to be matched up to the existing structure using ICP. Requires the initial
    * guess from odometry to already be very good.
    */
    bool enable_icp_;
    /**
     * If using ICP corrections, whether to store accumulate the corrected
     * transform. If this is set to false, the transform will reset every
     * iteration.
     */
    bool accumulate_icp_corrections_;

    bool verbose_;

    /// Subscriber settings.
    int pointcloud_queue_size_;
    int num_subscribers_tsdf_map_;

    /// Current transform corrections from ICP.
    voxblox::Transformation icp_corrected_transform_;

    /// ICP matcher
    std::shared_ptr<voxblox::ICP> icp_;

    // Maps and integrators.
    std::shared_ptr<voxblox::TsdfMap> tsdf_map_;
    std::unique_ptr<voxblox::TsdfIntegratorBase> tsdf_integrator_;

    /// Tools for broadcasting TFs.
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    /**
     * Global/map coordinate frame. Will always look up TF transforms to this
     * frame.
     */
    std::string world_frame_;

    /**
     * Name of the ICP corrected frame. Publishes TF and transform topic to this
     * if ICP on.
     */
    std::string icp_corrected_frame_;

    /// Name of the pose in the ICP correct Frame.
    std::string pose_corrected_frame_;

  public:
    TsdfServer(): Node("tsdf_server"),
        tf_broadcaster_(tf2_ros::TransformBroadcaster(this)) {
      //pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      //  "pointcloud", 10, std::bind(&TsdfServer::pointcloud_sub_callback, this, _1));
    }
    void processPointCloudMessageAndInsert(
      const sensor_msgs::msg::PointCloud2::Ptr& pointcloud_msg,
      const voxblox::Transformation& T_G_C, const bool is_freespace_pointcloud);

};
