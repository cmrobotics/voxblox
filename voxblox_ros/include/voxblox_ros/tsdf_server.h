#ifndef VOXBLOX_ROS_TSDF_SERVER_H_
#define VOXBLOX_ROS_TSDF_SERVER_H_

#include <memory>
#include <queue>
#include <string>

#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud.hpp>
//#include <ros/ros.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rclcpp/rclcpp.hpp>
#include <voxblox/alignment/icp.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/utils/color_maps.h>
#include <voxblox_msgs/srv/file_path.hpp>
#include <voxblox_msgs/msg/mesh.hpp>

#include "voxblox_ros/mesh_vis.h"
//#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/transformer.h"

namespace voxblox {

constexpr float kDefaultMaxIntensity = 100.0;

class TsdfServer : public rclcpp::Node {
 public:

  TsdfServer();
  TsdfServer(const TsdfMap::Config& config,
             const TsdfIntegratorBase::Config& integrator_config,
             const MeshIntegratorConfig& mesh_config);
  virtual ~TsdfServer() {}

  //void getServerConfigFromRosParam(const ros::NodeHandle& nh_private);

  void insertPointcloud(const sensor_msgs::msg::PointCloud2::Ptr& pointcloud);

  void insertFreespacePointcloud(
      const sensor_msgs::msg::PointCloud2::Ptr& pointcloud);

  virtual void processPointCloudMessageAndInsert(
      const sensor_msgs::msg::PointCloud2::Ptr& pointcloud_msg,
      const Transformation& T_G_C, const bool is_freespace_pointcloud);

  void integratePointcloud(const Transformation& T_G_C,
                           const Pointcloud& ptcloud_C, const Colors& colors,
                           const bool is_freespace_pointcloud = false);
  virtual void newPoseCallback(const Transformation& /*new_pose*/) {
    // Do nothing.
  }
  /**
  void publishAllUpdatedTsdfVoxels();
  void publishTsdfSurfacePoints();
  void publishTsdfOccupiedNodes();

  virtual void publishSlices();
  /// Incremental update.
  virtual void updateMesh();
  /// Batch update.
  virtual bool generateMesh();
  // Publishes all available pointclouds.
  virtual void publishPointclouds();
  // Publishes the complete map
  virtual void publishMap(bool reset_remote_map = false);
  virtual bool saveMap(const std::string& file_path);
  virtual bool loadMap(const std::string& file_path);

  bool clearMapCallback(std_srvs::srv::Empty::Request& request,           // NOLINT
                        std_srvs::srv::Empty::Response& response);        // NOLINT
  bool saveMapCallback(voxblox_msgs::srv::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::srv::FilePath::Response& response);  // NOLINT
  bool loadMapCallback(voxblox_msgs::srv::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::srv::FilePath::Response& response);  // NOLINT
  bool generateMeshCallback(std_srvs::srv::Empty::Request& request,       // NOLINT
                            std_srvs::srv::Empty::Response& response);    // NOLINT
  bool publishPointcloudsCallback(
      std_srvs::srv::Empty::Request& request,                             // NOLINT
      std_srvs::srv::Empty::Response& response);                          // NOLINT
  bool publishTsdfMapCallback(std_srvs::srv::Empty::Request& request,     // NOLINT
                              std_srvs::srv::Empty::Response& response);  // NOLINT

  void updateMeshEvent();//TODO: removed this, const voxblox::timing::TimerEvent& event);
  void publishMapEvent();//TODO: removed this, const voxblox::timing::TimerEvent& event);

  std::shared_ptr<TsdfMap> getTsdfMapPtr() { return tsdf_map_; }
  std::shared_ptr<const TsdfMap> getTsdfMapPtr() const { return tsdf_map_; }

  /// Accessors for setting and getting parameters.
  double getSliceLevel() const { return slice_level_; }
  void setSliceLevel(double slice_level) { slice_level_ = slice_level; }

  bool setPublishSlices() const { return publish_slices_; }
  void setPublishSlices(const bool publish_slices) {
    publish_slices_ = publish_slices;
  }

  void setWorldFrame(const std::string& world_frame) {
    world_frame_ = world_frame;
  }
  std::string getWorldFrame() const { return world_frame_; }

  /// CLEARS THE ENTIRE MAP!
  virtual void clear();

  /// Overwrites the layer with what's coming from the topic!
  void tsdfMapCallback(const voxblox_msgs::msg::Layer& layer_msg);

 protected:
  /**
   * Gets the next pointcloud that has an available transform to process from
   * the queue.
   */
  bool getNextPointcloudFromQueue(
      std::queue<sensor_msgs::msg::PointCloud2::Ptr>* queue,
      sensor_msgs::msg::PointCloud2::Ptr* pointcloud_msg, Transformation* T_G_C);

  /// Data subscribers.
  //rclcpp::Subscription<pcl::PointCloud<pcl::PointXYZI>> pointcloud_sub_;
  //rclcpp::Subscription<pcl::PointCloud<pcl::PointXYZI>> freespace_pointcloud_sub_;

  /// Publish markers for visualization.
  //rclcpp::Publisher<pcl::PointCloud<pcl::PointXYZI>> mesh_pub_;
  //rclcpp::Publisher<pcl::PointCloud<pcl::PointXYZI>> tsdf_pointcloud_pub_;
  //rclcpp::Publisher<pcl::PointCloud<pcl::PointXYZRGB>> surface_pointcloud_pub_;
  //rclcpp::Publisher<pcl::PointCloud<pcl::PointXYZI>> tsdf_slice_pub_;
  //rclcpp::Publisher<visualization_msgs::msg::MarkerArray> occupancy_marker_pub_;
  //rclcpp::Publisher<geometry_msgs::msg::TransformStamped> icp_transform_pub_;

  /// Publish the complete map for other nodes to consume.
  //rclcpp::Publisher<voxblox_msgs::msg::Layer> tsdf_map_pub_;

  /// Subscriber to subscribe to another node generating the map.
  //rclcpp::Subscription<voxblox_msgs::msg::Layer> tsdf_map_sub_;

  // Services. TODO: the generic types are wrong
  //rclcpp::Service<pcl::PointCloud<pcl::PointXYZI>> generate_mesh_srv_;
  //rclcpp::Service<pcl::PointCloud<pcl::PointXYZI>> clear_map_srv_;
  //rclcpp::Service<pcl::PointCloud<pcl::PointXYZI>> save_map_srv_;
  //rclcpp::Service<pcl::PointCloud<pcl::PointXYZI>> load_map_srv_;
  //rclcpp::Service<pcl::PointCloud<pcl::PointXYZI>> publish_pointclouds_srv_;
  //rclcpp::Service<pcl::PointCloud<pcl::PointXYZI>> publish_tsdf_map_srv_;

  /// Tools for broadcasting TFs.
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Timers.
  voxblox::timing::Timer update_mesh_timer_;
  voxblox::timing::Timer publish_map_timer_;

  bool verbose_;

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

  /// Delete blocks that are far from the system to help manage memory
  double max_block_distance_from_body_;

  /// Pointcloud visualization settings.
  double slice_level_;

  /// If the system should subscribe to a pointcloud giving points in freespace
  bool use_freespace_pointcloud_;

  /**
   * Mesh output settings. Mesh is only written to file if mesh_filename_ is
   * not empty.
   */
  std::string mesh_filename_;
  /// How to color the mesh.
  ColorMode color_mode_;

  /// Colormap to use for intensity pointclouds.
  std::shared_ptr<ColorMap> color_map_;

  /// Will throttle to this message rate.
  rclcpp::Duration min_time_between_msgs_;

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

  /// Subscriber settings.
  int pointcloud_queue_size_;
  int num_subscribers_tsdf_map_;

  // Maps and integrators.
  std::shared_ptr<TsdfMap> tsdf_map_;
  std::unique_ptr<TsdfIntegratorBase> tsdf_integrator_;

  /// ICP matcher
  std::shared_ptr<ICP> icp_;

  // Mesh accessories.
  std::shared_ptr<MeshLayer> mesh_layer_;
  std::unique_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator_;
  /// Optionally cached mesh message.
  voxblox_msgs::msg::Mesh cached_mesh_msg_;

  /**
   * Transformer object to keep track of either TF transforms or messages from
   * a transform topic.
   */
  Transformer transformer_;
  /**
   * Queue of incoming pointclouds, in case the transforms can't be immediately
   * resolved.
   */
  std::queue<sensor_msgs::msg::PointCloud2::Ptr> pointcloud_queue_;
  std::queue<sensor_msgs::msg::PointCloud2::Ptr> freespace_pointcloud_queue_;

  // Last message times for throttling input.
  rclcpp::Time last_msg_time_ptcloud_;
  rclcpp::Time last_msg_time_freespace_ptcloud_;

  /// Current transform corrections from ICP.
  Transformation icp_corrected_transform_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_TSDF_SERVER_H_
