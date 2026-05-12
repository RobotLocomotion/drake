#include "drake/planning/experimental/voxel_occupancy_map_internal.h"

namespace drake {
namespace planning {
namespace experimental {
namespace internal {

using common_robotics_utilities::parallelism::DegreeOfParallelism;
using voxelized_geometry_tools::OccupancyMap;

const OccupancyMap& GetInternalOccupancyMap(
    const VoxelOccupancyMap& occupancy_map) {
  return *reinterpret_cast<const OccupancyMap*>(
      occupancy_map.internal_representation());
}

// NOLINTNEXTLINE(runtime/references)
OccupancyMap& GetMutableInternalOccupancyMap(VoxelOccupancyMap& occupancy_map) {
  return *reinterpret_cast<OccupancyMap*>(
      occupancy_map.mutable_internal_representation());
}

}  // namespace internal
}  // namespace experimental
}  // namespace planning
}  // namespace drake
