#pragma once

#include <voxelized_geometry_tools/occupancy_map.hpp>

#include "drake/planning/experimental/voxel_occupancy_map.h"

namespace drake {
namespace planning {
namespace experimental {
namespace internal {
// Retrieve a const reference to the internal OccupancyMap of a
// VoxelOccupancyMap instance.
const voxelized_geometry_tools::OccupancyMap& GetInternalOccupancyMap(
    const VoxelOccupancyMap& occupancy_map);

// Retrieve a mutable reference to the internal OccupancyMap of a
// VoxelOccupancyMap instance. API takes mutable reference instead of pointer
// for consistency with non-mutable equivalent.
voxelized_geometry_tools::OccupancyMap& GetMutableInternalOccupancyMap(
    // NOLINTNEXTLINE(runtime/references)
    VoxelOccupancyMap& occupancy_map);

}  // namespace internal
}  // namespace experimental
}  // namespace planning
}  // namespace drake
