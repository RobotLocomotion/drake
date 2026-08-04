#pragma once

#include <voxelized_geometry_tools/occupancy_map.hpp>

#include "drake/planning/experimental/voxel_occupancy_map.h"

namespace drake {
namespace planning {
namespace experimental {
namespace internal {
// Retrieve a const reference to the internal OccupancyMap of a
// VoxelOccupancyMap instance.
inline const voxelized_geometry_tools::OccupancyMap& GetInternalOccupancyMap(
    const VoxelOccupancyMap& occupancy_map) {
  return *reinterpret_cast<const voxelized_geometry_tools::OccupancyMap*>(
      occupancy_map.internal_representation());
}

// Retrieve a mutable reference to the internal OccupancyMap of a
// VoxelOccupancyMap instance. API takes mutable reference instead of pointer
// for consistency with non-mutable equivalent.
inline voxelized_geometry_tools::OccupancyMap& GetMutableInternalOccupancyMap(
    // NOLINTNEXTLINE(runtime/references)
    VoxelOccupancyMap& occupancy_map) {
  return *reinterpret_cast<voxelized_geometry_tools::OccupancyMap*>(
      occupancy_map.mutable_internal_representation());
}

}  // namespace internal
}  // namespace experimental
}  // namespace planning
}  // namespace drake
