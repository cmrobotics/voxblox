#include <voxblox_ros/tsdf_server.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <voxblox_ros/conversions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <pcl_conversions/pcl_conversions.h>
#include <voxblox/utils/timing.h>
//#include <voxblox_ros/kindr_tf.h>
//#include <voxblox_ros/kindr_msg.h>
#include <tf2_eigen/tf2_eigen.h>

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

      Eigen::Isometry3d icp_isometry_3d(icp_corrected_transform_.cast<double>().getTransformationMatrix());
      geometry_msgs::msg::TransformStamped icp_geometry_msg = tf2::eigenToTransform(icp_isometry_3d);
      icp_geometry_msg.header.stamp    = pointcloud_msg->header.stamp;
      icp_geometry_msg.header.frame_id = world_frame_;
      icp_geometry_msg.child_frame_id  = icp_corrected_frame_;
      tf_broadcaster_.sendTransform(icp_geometry_msg);

      Eigen::Isometry3d pose_isometry_3d(T_G_C.cast<double>().getTransformationMatrix());
      geometry_msgs::msg::TransformStamped pose_geometry_msg = tf2::eigenToTransform(pose_isometry_3d);
      pose_geometry_msg.header.stamp    = pointcloud_msg->header.stamp;
      pose_geometry_msg.header.frame_id = icp_corrected_frame_;
      pose_geometry_msg.child_frame_id  = pose_corrected_frame_;
      tf_broadcaster_.sendTransform(pose_geometry_msg);

      icp_transform_pub_->publish(icp_geometry_msg);
      icp_timer.Stop();
    }


    if (verbose_) {
      //TODO ROS_INFO("Integrating a pointcloud with %lu points.", points_C.size());
    }


    //ros::WallTime start = ros::WallTime::now();
    integratePointcloud(T_G_C_refined, points_C, colors, is_freespace_pointcloud);
    //ros::WallTime end = ros::WallTime::now();
    if (verbose_) {
      //TODO ROS_INFO("Finished integrating in %f seconds, have %lu blocks.",
      //         (end - start).toSec(),
      //        tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks());
    }
    /**
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

void TsdfServer::integratePointcloud(const voxblox::Transformation& T_G_C,
                                    const voxblox::Pointcloud& ptcloud_C,
                                    const voxblox::Colors& colors,
                                    const bool is_freespace_pointcloud) {
  //CHECK_EQ(ptcloud_C.size(), colors.size());
  //tsdf_integrator_->integratePointCloud(T_G_C, ptcloud_C, colors,
  //                                    is_freespace_pointcloud);
}


