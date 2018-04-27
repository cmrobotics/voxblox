#include "voxblox/integrator/thermal_integrator.h"

#include "voxblox/utils/distance_utils.h"

namespace voxblox {

ThermalIntegrator::ThermalIntegrator(const Layer<TsdfVoxel>& tsdf_layer,
                                     Layer<ThermalVoxel>* thermal_layer)
    : tsdf_layer_(tsdf_layer), thermal_layer_(thermal_layer) {}

void ThermalIntegrator::addThermalBearingVectors(
    const Point& origin, const Pointcloud& bearing_vectors,
    const std::vector<float>& temperatures) {
  CHECK_EQ(bearing_vectors.size(), temperatures.size())
      << "Temperature and bearing vector size does not match!";
  constexpr FloatingPoint max_distance = 10.0;
  const FloatingPoint temperature_prop_distance =
      4 * tsdf_layer_.voxel_size();  // This equates to 2 * voxel_size
                                     // truncation distance.

  for (size_t i = 0; i < bearing_vectors.size(); ++i) {
    Point surface_intersection = Point::Zero();
    // Cast ray from the origin in the direction of the bearing vector until
    // finding an intersection with a surface.
    bool success = getSurfaceDistanceAlongRay<TsdfVoxel>(
        tsdf_layer_, origin, bearing_vectors[i], max_distance,
        &surface_intersection);

    if (!success) {
      continue;
    }

    // Now look up the matching voxels in the thermal layer and mark them.
    // Let's just start with 1.
    Block<ThermalVoxel>::Ptr block_ptr =
        thermal_layer_->allocateBlockPtrByCoordinates(surface_intersection);
    ThermalVoxel& voxel =
        block_ptr->getVoxelByCoordinates(surface_intersection);
    /*if (voxel.observations < 1) {
      voxel.temperature = temperatures[i];
      voxel.observations = 1;
    } else { */
      voxel.temperature =
          (voxel.observations * voxel.temperature + temperatures[i]) /
          (voxel.observations + 1);
      voxel.observations++;
    //}
    if (voxel.observations > 100) {
      voxel.observations = 100;
    }
  }
}

}  // namespace voxblox
