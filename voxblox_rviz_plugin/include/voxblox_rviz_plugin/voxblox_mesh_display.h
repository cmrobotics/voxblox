#ifndef VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H_
#define VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H_

#include <memory>

#include <rviz_common/message_filter_display.h>
#include <voxblox_msgs/msg/mesh.hpp>

#include "voxblox_rviz_plugin/voxblox_mesh_visual.h"

namespace voxblox_rviz_plugin {

class VoxbloxMeshVisual;

class VoxbloxMeshDisplay
    : public rviz::MessageFilterDisplay<voxblox_msgs::msg::Mesh> {
  Q_OBJECT
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VoxbloxMeshDisplay();
  virtual ~VoxbloxMeshDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private:
  void processMessage(const voxblox_msgs::msg::Mesh::ConstPtr& msg);

  std::unique_ptr<VoxbloxMeshVisual> visual_;
};

}  // namespace voxblox_rviz_plugin

#endif  // VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H_
