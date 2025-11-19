#pragma once

#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "drake/common/drake_assert.h"
#include "drake/math/rigid_transform.h"
#include "drake/planning/dev/voxel_signed_distance_field.h"

namespace drake {
namespace planning {

class VoxelOccupancyMap {
 public:
  /// Default constructor creates an empty VoxelOccupancyMap.
  VoxelOccupancyMap();

  /// Construct a VoxelOccupancyMap filled with cells of `default_occupancy` and
  /// `default_object_id`. `parent_body_name` specifies the name of the parent
  /// body, and `X_PG` specifies the pose of the origin of the voxel grid
  /// relative to the parent body frame. `grid_dimensions` specifies the nominal
  /// dimensions of the voxel grid, and `grid_resolution` specifies the size of
  /// an individual voxel. If you specify dimensions that are not evenly
  /// divisible by `grid_resolution`, you will get a larger grid
  /// with num_cells = ceil(dimension/resolution).
  VoxelOccupancyMap(const std::string& parent_body_name,
                    const math::RigidTransformd& X_PG,
                    const Eigen::Vector3d& grid_dimensions,
                    double grid_resolution, float default_occupancy);

  /// Construct a VoxelOccupancyMap filled with cells of `default_occupancy` and
  /// `default_object_id`. `parent_body_name` specifies the name of the parent
  /// body, and `X_PG` specifies the pose of the origin of the voxel grid
  /// relative to the parent body frame. `grid_sizes` specifies the number of
  /// voxels for each axis of the voxel grid, and `grid_resolution` specifies
  /// the size of an individual voxel.
  VoxelOccupancyMap(const std::string& parent_body_name,
                    const math::RigidTransformd& X_PG,
                    const Eigen::Matrix<int64_t, 3, 1>& grid_sizes,
                    double grid_resolution, float default_occupancy);

  /// VoxelOccupancyMap provides copy, move, and assignment.
  VoxelOccupancyMap(const VoxelOccupancyMap& other);
  VoxelOccupancyMap(VoxelOccupancyMap&& other);
  VoxelOccupancyMap& operator=(const VoxelOccupancyMap& other);
  VoxelOccupancyMap& operator=(VoxelOccupancyMap&& other);

  /// Construct a voxelized signed distance field from `this` using the provided
  /// parameters `parameters`.
  VoxelSignedDistanceField ExportSignedDistanceField(
      const VoxelSignedDistanceField::GenerationParameters& parameters = {})
      const;

  /// Get the name of the parent body frame.
  const std::string& parent_body_name() const;

  /// Returns true if empty. A VoxelOccupancyMap will be empty if it is in the
  /// default constructed state or has been moved-from.
  bool is_empty() const;

  // Internal-only, access to the internal voxel grid.
  const void* internal_representation() const {
    DRAKE_DEMAND(internal_representation_ != nullptr);
    return internal_representation_.get();
  }

  // Internal-only, mutable access to the internal voxel grid.
  void* mutable_internal_representation() {
    DRAKE_DEMAND(internal_representation_ != nullptr);
    return internal_representation_.get();
  }

 private:
  void InitializeEmpty();

  // Opaque handle to the internal voxel grid representation.
  // Note that while a shared_ptr is used here for type erasure, the internal
  // representation is never shared between VoxelOccupancyMap objects.
  std::shared_ptr<void> internal_representation_;
};

}  // namespace planning
}  // namespace drake
