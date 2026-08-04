#pragma once

#include <voxelized_geometry_tools/tagged_object_occupancy_map.hpp>

#include "drake/planning/experimental/voxel_tagged_object_occupancy_map.h"

namespace drake {
namespace planning {
namespace experimental {
namespace internal {
// Retrieve a const reference to the internal TaggedObjectOccupancyMap of a
// VoxelTaggedObjectOccupancyMap instance.
inline const voxelized_geometry_tools::TaggedObjectOccupancyMap&
GetInternalTaggedObjectOccupancyMap(
    const VoxelTaggedObjectOccupancyMap& occupancy_map) {
  using voxelized_geometry_tools::TaggedObjectOccupancyMap;
  return *reinterpret_cast<const TaggedObjectOccupancyMap*>(
      occupancy_map.internal_representation());
}

// Retrieve a mutable reference to the internal TaggedObjectOccupancyMap of a
// VoxelTaggedObjectOccupancyMap instance. API takes mutable reference instead
// of pointer for consistency with non-mutable equivalent.
inline voxelized_geometry_tools::TaggedObjectOccupancyMap&
GetMutableInternalTaggedObjectOccupancyMap(
    // NOLINTNEXTLINE(runtime/references)
    VoxelTaggedObjectOccupancyMap& occupancy_map) {
  return *reinterpret_cast<voxelized_geometry_tools::TaggedObjectOccupancyMap*>(
      occupancy_map.mutable_internal_representation());
}

}  // namespace internal
}  // namespace experimental
}  // namespace planning
}  // namespace drake
