#pragma once

#include <voxelized_geometry_tools/collision_map.hpp>

#include "drake/planning/dev/voxel_collision_map.h"

namespace drake {
namespace planning {
namespace internal {

const voxelized_geometry_tools::CollisionMap& GetInternalCollisionMap(
    const VoxelCollisionMap& collision_map);

voxelized_geometry_tools::CollisionMap& GetMutableInternalCollisionMap(
    // NOLINTNEXTLINE(runtime/references)
    VoxelCollisionMap& collision_map);

}  // namespace internal
}  // namespace planning
}  // namespace drake
