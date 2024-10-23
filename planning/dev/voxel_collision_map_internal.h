#pragma once

#include <voxelized_geometry_tools/collision_map.hpp>

#include "drake/planning/dev/voxel_collision_map.h"

namespace drake {
namespace planning {
namespace internal {
// Retrieve a const reference to the internal CollisionMap of a
// VoxelCollisionMap instance.
const voxelized_geometry_tools::CollisionMap& GetInternalCollisionMap(
    const VoxelCollisionMap& collision_map);

// Retrieve a mutable reference to the internal CollisionMap of a
// VoxelCollisionMap instance. API takes mutable reference instead of pointer
// for consistency with non-mutable equivalent.
voxelized_geometry_tools::CollisionMap& GetMutableInternalCollisionMap(
    // NOLINTNEXTLINE(runtime/references)
    VoxelCollisionMap& collision_map);

}  // namespace internal
}  // namespace planning
}  // namespace drake
