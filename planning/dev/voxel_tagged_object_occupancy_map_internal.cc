#include "drake/planning/dev/voxel_tagged_object_occupancy_map_internal.h"

#include <common_robotics_utilities/parallelism.hpp>

namespace drake {
namespace planning {
namespace internal {

using voxelized_geometry_tools::TaggedObjectOccupancyMap;

const TaggedObjectOccupancyMap& GetInternalTaggedObjectOccupancyMap(
    const VoxelTaggedObjectOccupancyMap& occupancy_map) {
  return *reinterpret_cast<const TaggedObjectOccupancyMap*>(
      occupancy_map.internal_representation());
}

TaggedObjectOccupancyMap& GetMutableInternalTaggedObjectOccupancyMap(
    // NOLINTNEXTLINE(runtime/references)
    VoxelTaggedObjectOccupancyMap& occupancy_map) {
  return *reinterpret_cast<TaggedObjectOccupancyMap*>(
      occupancy_map.mutable_internal_representation());
}

}  // namespace internal
}  // namespace planning
}  // namespace drake
