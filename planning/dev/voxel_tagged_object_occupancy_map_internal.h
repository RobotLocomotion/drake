#pragma once

#include <voxelized_geometry_tools/tagged_object_occupancy_map.hpp>

#include "drake/planning/dev/voxel_tagged_object_occupancy_map.h"

namespace drake {
namespace planning {
namespace internal {
// Retrieve a const reference to the internal TaggedObjectOccupancyMap of a
// VoxelTaggedObjectOccupancyMap instance.
const voxelized_geometry_tools::TaggedObjectOccupancyMap&
GetInternalTaggedObjectOccupancyMap(
    const VoxelTaggedObjectOccupancyMap& occupancy_map);

// Retrieve a mutable reference to the internal TaggedObjectOccupancyMap of a
// VoxelTaggedObjectOccupancyMap instance. API takes mutable reference instead
// of pointer for consistency with non-mutable equivalent.
voxelized_geometry_tools::TaggedObjectOccupancyMap&
GetMutableInternalTaggedObjectOccupancyMap(
    // NOLINTNEXTLINE(runtime/references)
    VoxelTaggedObjectOccupancyMap& occupancy_map);

}  // namespace internal
}  // namespace planning
}  // namespace drake
