#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/planning/dev/voxel_signed_distance_field.h"

namespace drake {
namespace planning {

class VoxelCollisionMap {
 public:
  VoxelCollisionMap();

  /// VoxelCollisionMap provides copy, move, and assignment.
  VoxelCollisionMap(const VoxelCollisionMap& other);
  VoxelCollisionMap(VoxelCollisionMap&& other);
  VoxelCollisionMap& operator=(const VoxelCollisionMap& other);
  VoxelCollisionMap& operator=(VoxelCollisionMap&& other);

  /// Construct a voxelized signed distance field from `this` using the provided
  /// parameters `parameters`.
  VoxelSignedDistanceField ExportSignedDistanceField(
      const VoxelSignedDistanceField::GenerationParameters& parameters = {})
      const;

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
  // representation is never shared between VoxelCollisionMap objects.
  std::shared_ptr<void> internal_representation_;
};

}  // namespace planning
}  // namespace drake
