#include "drake/planning/dev/voxel_tagged_object_collision_map_internal.h"

#include <common_robotics_utilities/parallelism.hpp>

namespace drake {
namespace planning {
namespace internal {

using voxelized_geometry_tools::TaggedObjectCollisionMap;

const TaggedObjectCollisionMap& GetInternalTaggedObjectCollisionMap(
    const VoxelTaggedObjectCollisionMap& collision_map) {
  return *reinterpret_cast<const TaggedObjectCollisionMap*>(
      collision_map.internal_representation());
}

TaggedObjectCollisionMap& GetMutableInternalTaggedObjectCollisionMap(
    // NOLINTNEXTLINE(runtime/references)
    VoxelTaggedObjectCollisionMap& collision_map) {
  return *reinterpret_cast<TaggedObjectCollisionMap*>(
      collision_map.mutable_internal_representation());
}

}  // namespace internal
}  // namespace planning
}  // namespace drake
