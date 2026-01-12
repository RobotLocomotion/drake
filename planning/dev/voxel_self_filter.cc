#include "drake/planning/dev/voxel_self_filter.h"

#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/voxel_grid.hpp>

#include "drake/planning/dev/voxel_occupancy_map_internal.h"
#include "drake/planning/dev/voxel_self_filter_internal.h"
#include "drake/planning/dev/voxel_tagged_object_occupancy_map_internal.h"

namespace drake {
namespace planning {

using multibody::BodyIndex;
using voxelized_geometry_tools::OccupancyCell;
using voxelized_geometry_tools::TaggedObjectOccupancyCell;

void SelfFilter(const SphereRobotModelCollisionChecker& collision_checker,
                const Eigen::VectorXd& q, const double padding,
                const BodyIndex grid_body_index,
                VoxelOccupancyMap* const occupancy_map,
                const Parallelism parallelism,
                const std::optional<int> context_number) {
  DRAKE_THROW_UNLESS(occupancy_map != nullptr);
  auto& internal_occupancy_map =
      internal::GetMutableInternalOccupancyMap(*occupancy_map);
  const OccupancyCell empty_cell(0.0f);
  return internal::SelfFilter(collision_checker, q, padding, grid_body_index,
                              &internal_occupancy_map, empty_cell, parallelism,
                              context_number);
}

void SelfFilter(const SphereRobotModelCollisionChecker& collision_checker,
                const Eigen::VectorXd& q, const double padding,
                const BodyIndex grid_body_index,
                VoxelTaggedObjectOccupancyMap* const occupancy_map,
                const Parallelism parallelism,
                const std::optional<int> context_number) {
  DRAKE_THROW_UNLESS(occupancy_map != nullptr);
  auto& internal_occupancy_map =
      internal::GetMutableInternalTaggedObjectOccupancyMap(*occupancy_map);
  const TaggedObjectOccupancyCell empty_cell(0.0f, 0u);
  return internal::SelfFilter(collision_checker, q, padding, grid_body_index,
                              &internal_occupancy_map, empty_cell, parallelism,
                              context_number);
}

}  // namespace planning
}  // namespace drake
