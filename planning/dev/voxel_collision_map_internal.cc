#include "drake/planning/dev/voxel_collision_map_internal.h"

namespace drake {
namespace planning {
namespace internal {

using common_robotics_utilities::parallelism::DegreeOfParallelism;
using voxelized_geometry_tools::CollisionMap;

const CollisionMap& GetInternalCollisionMap(
    const VoxelCollisionMap& collision_map) {
  return *reinterpret_cast<const CollisionMap*>(
      collision_map.internal_representation());
}

CollisionMap& GetMutableInternalCollisionMap(VoxelCollisionMap& collision_map) {
  return *reinterpret_cast<CollisionMap*>(
      collision_map.mutable_internal_representation());
}

}  // namespace internal
}  // namespace planning
}  // namespace drake
