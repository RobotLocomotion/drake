#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "drake/common/drake_assert.h"
#include "drake/math/rigid_transform.h"
#include "drake/planning/dev/voxel_signed_distance_field.h"

namespace drake {
namespace planning {

class VoxelTaggedObjectOccupancyMap {
 public:
  /// Default constructor creates an empty VoxelOccupancyMap.
  VoxelTaggedObjectOccupancyMap();

  /// Construct a VoxelTaggedObjectOccupancyMap filled with cells of
  /// `default_occupancy` and `default_object_id`. `parent_body_name` specifies
  /// the name of the parent body, and `X_PG` specifies the pose of the origin
  /// of the voxel grid relative to the parent body frame. `grid_dimensions`
  /// specifies the nominal dimensions of the voxel grid, and `grid_resolution`
  /// specifies the size of an individual voxel. If you specify dimensions that
  /// are not evenly divisible by `grid_resolution`, you will get a larger grid
  /// with num_cells = ceil(dimension/resolution).
  VoxelTaggedObjectOccupancyMap(const std::string& parent_body_name,
                                const math::RigidTransformd& X_PG,
                                const Eigen::Vector3d& grid_dimensions,
                                double grid_resolution, float default_occupancy,
                                uint32_t default_object_id);

  /// Construct a VoxelTaggedObjectOccupancyMap filled with cells of
  /// `default_occupancy` and `default_object_id`. `parent_body_name` specifies
  /// the name of the parent body, and `X_PG` specifies the pose of the origin
  /// of the voxel grid relative to the parent body frame. `grid_sizes`
  /// specifies the number of voxels for each axis of the voxel grid, and
  /// `grid_resolution` specifies the size of an individual voxel.
  VoxelTaggedObjectOccupancyMap(const std::string& parent_body_name,
                                const math::RigidTransformd& X_PG,
                                const Eigen::Matrix<int64_t, 3, 1>& grid_sizes,
                                double grid_resolution, float default_occupancy,
                                uint32_t default_object_id);

  /// VoxelTaggedObjectOccupancyMap provides copy, move, and assignment.
  VoxelTaggedObjectOccupancyMap(const VoxelTaggedObjectOccupancyMap& other);
  VoxelTaggedObjectOccupancyMap(VoxelTaggedObjectOccupancyMap&& other);
  VoxelTaggedObjectOccupancyMap& operator=(
      const VoxelTaggedObjectOccupancyMap& other);
  VoxelTaggedObjectOccupancyMap& operator=(
      VoxelTaggedObjectOccupancyMap&& other);

  /// Construct a voxelized signed distance field from `this` using the provided
  /// parameters `parameters`. The objects to include are specified by
  /// `objects_to_include`; if left empty all objects will be included.
  VoxelSignedDistanceField ExportSignedDistanceField(
      const std::vector<uint32_t>& objects_to_include = {},
      const VoxelSignedDistanceField::GenerationParameters& parameters = {})
      const;

  /// Get the name of the parent body frame.
  const std::string& parent_body_name() const;

  /// Returns true if empty. A VoxelTaggedObjectOccupancyMap will be empty if it
  /// is in the default constructed state or has been moved-from.
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
  // representation is never shared between VoxelTaggedObjectOccupancyMap
  // objects.
  std::shared_ptr<void> internal_representation_;
};

}  // namespace planning
}  // namespace drake
