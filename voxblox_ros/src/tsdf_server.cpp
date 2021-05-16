#include <voxblox_ros/tsdf_server.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <voxblox_ros/conversions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <pcl_conversions/pcl_conversions.h>
#include <voxblox/utils/timing.h>
#include <voxblox_ros/kindr_tf.h>
#include <voxblox_ros/kindr_msg.h>

void TsdfServer::processPointCloudMessageAndInsert(
    const sensor_msgs::msg::PointCloud2::Ptr& pointcloud_msg,
    const voxblox::Transformation& T_G_C, const bool is_freespace_pointcloud) {

  // Convert the PCL pointcloud into our awesome format.
  // Horrible hack fix to fix color parsing colors in PCL.
  bool color_pointcloud = false;
  bool has_intensity = false;
  for (size_t d = 0; d < pointcloud_msg->fields.size(); ++d) {
    if (pointcloud_msg->fields[d].name == std::string("rgb")) {
      pointcloud_msg->fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
      color_pointcloud = true;
    } else if (pointcloud_msg->fields[d].name == std::string("intensity")) {
      has_intensity = true;
    }
  }
  voxblox::Pointcloud points_C;
  voxblox::Colors colors;
  voxblox::timing::Timer ptcloud_timer("ptcloud_preprocess");

  // Convert differently depending on RGB or I type.
  if (color_pointcloud) {
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    voxblox::convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  } else if (has_intensity) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  } else {
    pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  }
  ptcloud_timer.Stop();



  voxblox::Transformation T_G_C_refined = T_G_C;
  if (enable_icp_) {
    voxblox::timing::Timer icp_timer("icp");
    if (!accumulate_icp_corrections_) {
      icp_corrected_transform_.setIdentity();
    }
    static voxblox::Transformation T_offset;
    const size_t num_icp_updates =
        icp_->runICP(tsdf_map_->getTsdfLayer(), points_C,
                     icp_corrected_transform_ * T_G_C, &T_G_C_refined);
    if (verbose_) {
      //TODO ROS_INFO("ICP refinement performed %zu successful update steps",
      //         num_icp_updates);
    }
    icp_corrected_transform_ = T_G_C_refined * T_G_C.inverse();

    if (!icp_->refiningRollPitch()) {
      // its already removed internally but small floating point errors can
      // build up if accumulating transforms
      voxblox::Transformation::Vector6 T_vec = icp_corrected_transform_.log();
      T_vec[3] = 0.0;
      T_vec[4] = 0.0;
      icp_corrected_transform_ = voxblox::Transformation::exp(T_vec);
    }

    // Publish transforms as both TF and message.
    tf2::Transform icp_tf_msg, pose_tf_msg;
    geometry_msgs::msg::TransformStamped transform_msg;

    tf::transformKindrToTF(icp_corrected_transform_.cast<double>(),
                           &icp_tf_msg);
    tf::transformKindrToTF(T_G_C.cast<double>(), &pose_tf_msg);
    tf::transformKindrToMsg(icp_corrected_transform_.cast<double>(),
                            &transform_msg.transform);

    geometry_msgs::msg::TransformStamped world_to_icp;
    world_to_icp.header.stamp = pointcloud_msg->header.stamp;
    world_to_icp.header.frame_id = world_frame_;
    world_to_icp.child_frame_id  = icp_corrected_frame_;
    //world_to_icp.transform.rotation = icp_tf_msg.getRotation();
    //world_to_icp.transform.translation = icp_tf_msg.getOrigin();
    tf_broadcaster_.sendTransform(world_to_icp);
 /**
    tf_broadcaster_.sendTransform(
        geometry_msgs::msg::TransformStamped(
            pose_tf_msg, pointcloud_msg->header.stamp,
            icp_corrected_frame_, pose_corrected_frame_));

    transform_msg.header.frame_id = world_frame_;
    transform_msg.child_frame_id = icp_corrected_frame_;
    icp_transform_pub_.publish(transform_msg);

    icp_timer.Stop();

  if (verbose_) {
    //TODO ROS_INFO("Integrating a pointcloud with %lu points.", points_C.size());
  }

  ros::WallTime start = ros::WallTime::now();
  integratePointcloud(T_G_C_refined, points_C, colors, is_freespace_pointcloud);
  ros::WallTime end = ros::WallTime::now();
  if (verbose_) {
    //TODO ROS_INFO("Finished integrating in %f seconds, have %lu blocks.",
    //         (end - start).toSec(),
    //        tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks());
  }

  timing::Timer block_remove_timer("remove_distant_blocks");
  tsdf_map_->getTsdfLayerPtr()->removeDistantBlocks(
      T_G_C.getPosition(), max_block_distance_from_body_);
  mesh_layer_->clearDistantMesh(T_G_C.getPosition(),
                                max_block_distance_from_body_);
  block_remove_timer.Stop();

  // Callback for inheriting classes.
  newPoseCallback(T_G_C);
  */
  }

}