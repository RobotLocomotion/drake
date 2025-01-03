#pragma once

#include <voxelized_geometry_tools/tagged_object_collision_map.hpp>

#include "drake/planning/dev/voxel_tagged_object_collision_map.h"

namespace drake {
namespace planning {
namespace internal {
// Retrieve a const reference to the internal TaggedObjectCollisionMap of a
// VoxelTaggedObjectCollisionMap instance.
const voxelized_geometry_tools::TaggedObjectCollisionMap&
GetInternalTaggedObjectCollisionMap(
    const VoxelTaggedObjectCollisionMap& collision_map);

// Retrieve a mutable reference to the internal TaggedObjectCollisionMap of a
// VoxelTaggedObjectCollisionMap instance. API takes mutable reference instead
// of pointer for consistency with non-mutable equivalent.
voxelized_geometry_tools::TaggedObjectCollisionMap&
GetMutableInternalTaggedObjectCollisionMap(
    // NOLINTNEXTLINE(runtime/references)
    VoxelTaggedObjectCollisionMap& collision_map);

}  // namespace internal
}  // namespace planning
}  // namespace drake
