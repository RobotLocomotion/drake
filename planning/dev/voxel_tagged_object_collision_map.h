#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/planning/dev/voxel_signed_distance_field.h"

namespace drake {
namespace planning {

class VoxelTaggedObjectCollisionMap {
 public:
  VoxelTaggedObjectCollisionMap();

  /// VoxelTaggedObjectCollisionMap provides copy, move, and assignment.
  VoxelTaggedObjectCollisionMap(const VoxelTaggedObjectCollisionMap& other);
  VoxelTaggedObjectCollisionMap(VoxelTaggedObjectCollisionMap&& other);
  VoxelTaggedObjectCollisionMap& operator=(
      const VoxelTaggedObjectCollisionMap& other);
  VoxelTaggedObjectCollisionMap& operator=(
      VoxelTaggedObjectCollisionMap&& other);

  /// Construct a voxelized signed distance field from `this` using the provided
  /// parameters `parameters`. The objects to include are specified by
  /// `objects_to_include`; if left empty all objects will be included.
  VoxelSignedDistanceField ExportSignedDistanceField(
      const std::vector<uint32_t>& objects_to_include = {},
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
  // representation is never shared between VoxelTaggedObjectCollisionMap
  // objects.
  std::shared_ptr<void> internal_representation_;
};

}  // namespace planning
}  // namespace drake
