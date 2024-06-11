#include "drake/planning/dev/voxel_self_filter.h"

#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/voxel_grid.hpp>

#include "drake/planning/dev/voxel_self_filter_internal.h"

namespace drake {
namespace planning {

using multibody::BodyIndex;
using voxelized_geometry_tools::CollisionCell;
using voxelized_geometry_tools::TaggedObjectCollisionCell;

void SelfFilter(const SphereRobotModelCollisionChecker& collision_checker,
                const Eigen::VectorXd& q, const double padding,
                const BodyIndex grid_body_index,
                voxelized_geometry_tools::CollisionMap* const collision_map,
                const Parallelism parallelism,
                const std::optional<int> context_number) {
  DRAKE_THROW_UNLESS(collision_map != nullptr);
  const CollisionCell empty_cell(0.0f, 0u);
  return internal::SelfFilter(collision_checker, q, padding, grid_body_index,
                              collision_map, empty_cell, parallelism,
                              context_number);
}

void SelfFilter(
    const SphereRobotModelCollisionChecker& collision_checker,
    const Eigen::VectorXd& q, const double padding,
    const BodyIndex grid_body_index,
    voxelized_geometry_tools::TaggedObjectCollisionMap* const collision_map,
    const Parallelism parallelism, const std::optional<int> context_number) {
  DRAKE_THROW_UNLESS(collision_map != nullptr);
  const TaggedObjectCollisionCell empty_cell(0.0f, 0u, 0u, 0u);
  return internal::SelfFilter(collision_checker, q, padding, grid_body_index,
                              collision_map, empty_cell, parallelism,
                              context_number);
}

}  // namespace planning
}  // namespace drake
