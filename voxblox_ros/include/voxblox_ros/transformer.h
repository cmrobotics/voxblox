#ifndef VOXBLOX_ROS_TRANSFORMER_H_
#define VOXBLOX_ROS_TRANSFORMER_H_

#include <string>

//#include <geometry_msgs/msg/transform_stamped.h>
//#include <tf2_ros/transform_listener.h>
//#include <rclcpp/time.hpp>
//#include <voxblox/core/common.h>
#include <rclcpp/rclcpp.hpp>


namespace voxblox {

/**
 * Class that binds to either the TF tree or resolves transformations from the
 * ROS parameter server, depending on settings loaded from ROS params.
 */
class Transformer {
 public:
  //vEIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Transformer();

  //bool lookupTransform(const std::string& from_frame,
   //                    const std::string& to_frame, const rclcpp::Time& timestamp,
    //                   Transformation* transform);

  //void transformCallback(const geometry_msgs::msg::TransformStamped& transform_msg);

 private:
  //bool lookupTransformTf(const std::string& from_frame,
   //                      const std::string& to_frame,
  //                       const rclcpp::Time& timestamp, Transformation* transform);

  //bool lookupTransformQueue(const rclcpp::Time& timestamp,
  //                          Transformation* transform);

  /**
   * Global/map coordinate frame. Will always look up TF transforms to this
   * frame.
   */
  //std::string world_frame_;
  /// If set, overwrite sensor frame with this value. If empty, unused.
  //std::string sensor_frame_;
  /**
   * Whether to use TF transform resolution (true) or fixed transforms from
   * parameters and transform topics (false).
   */
  //bool use_tf_transforms_;
  //int64_t timestamp_tolerance_ns_;
  /**
   * B is the body frame of the robot, C is the camera/sensor frame creating
   * the pointclouds, and D is the 'dynamic' frame; i.e., incoming messages
   * are assumed to be T_G_D.
   */
  //Transformation T_B_C_;
  //Transformation T_B_D_;
  /**
   * If we use topic transforms, we have 2 parts: a dynamic transform from a
   * topic and a static transform from parameters.
   * Static transform should be T_G_D (where D is whatever sensor the
   * dynamic coordinate frame is in) and the static should be T_D_C (where
   * C is the sensor frame that produces the depth data). It is possible to
   * specify T_C_D and set invert_static_transform to true.
   */

  /**
   * To be replaced (at least optionally) with odometry + static transform
   * from IMU to visual frame.
   */
  //tf2_ros::TransformListener tf_listener_;

  // l Only used if use_tf_transforms_ set to false.
  //rclcpp::Subscription<geometry_msgs::msg::TransformStamped> transform_sub_;

  // l Transform queue, used only when use_tf_transforms is false.
  //AlignedDeque<geometry_msgs::msg::TransformStamped> transform_queue_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_TRANSFORMER_H_
