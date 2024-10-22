#pragma once

#include <voxelized_geometry_tools/tagged_object_collision_map.hpp>

#include "drake/planning/dev/voxel_tagged_object_collision_map.h"

namespace drake {
namespace planning {
namespace internal {

const voxelized_geometry_tools::TaggedObjectCollisionMap&
GetInternalTaggedObjectCollisionMap(
    const VoxelTaggedObjectCollisionMap& collision_map);

voxelized_geometry_tools::TaggedObjectCollisionMap&
GetMutableInternalTaggedObjectCollisionMap(
    VoxelTaggedObjectCollisionMap& collision_map);

}  // namespace internal
}  // namespace planning
}  // namespace drake
