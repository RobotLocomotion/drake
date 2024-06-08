#include "planning/voxelized_environment_builder.h"

#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/print.hpp>
#include <voxelized_geometry_tools/collision_map.hpp>
#include <voxelized_geometry_tools/tagged_object_collision_map.hpp>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace anzu {
namespace planning {
voxelized_geometry_tools::CollisionMap BuildCollisionMap(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& plant_context,
    const std::unordered_set<drake::geometry::GeometryId>&
        geometries_to_ignore,
    const std::string& parent_body_name, const Eigen::Isometry3d& X_BG,
    const Eigen::Vector3d& grid_size, const double grid_resolution,
    const std::optional<drake::multibody::BodyIndex>&
        override_parent_body_index,
    const Parallelism parallelism) {
  const common_robotics_utilities::voxel_grid::GridSizes grid_sizes(
      grid_resolution, grid_size.x(), grid_size.y(), grid_size.z());
  const voxelized_geometry_tools::CollisionCell empty_cell(0.0f);
  voxelized_geometry_tools::CollisionMap collision_map(
      X_BG, parent_body_name, grid_sizes, empty_cell);
  FillCollisionMap(
      plant, plant_context, geometries_to_ignore, &collision_map,
      override_parent_body_index, parallelism);
  return collision_map;
}

voxelized_geometry_tools::TaggedObjectCollisionMap
BuildTaggedObjectCollisionMap(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& plant_context,
    const std::unordered_set<drake::geometry::GeometryId>&
        geometries_to_ignore,
    const std::string& parent_body_name, const Eigen::Isometry3d& X_BG,
    const Eigen::Vector3d& grid_size, const double grid_resolution,
    const std::optional<drake::multibody::BodyIndex>&
        override_parent_body_index,
    const Parallelism parallelism) {
  const common_robotics_utilities::voxel_grid::GridSizes grid_sizes(
      grid_resolution, grid_size.x(), grid_size.y(), grid_size.z());
  const voxelized_geometry_tools::TaggedObjectCollisionCell empty_cell;
  voxelized_geometry_tools::TaggedObjectCollisionMap collision_map(
      X_BG, parent_body_name, grid_sizes, empty_cell);
  FillTaggedObjectCollisionMap(
      plant, plant_context, geometries_to_ignore, &collision_map,
      override_parent_body_index, parallelism);
  return collision_map;
}

void FillCollisionMap(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& plant_context,
    const std::unordered_set<drake::geometry::GeometryId>&
        geometries_to_ignore,
    voxelized_geometry_tools::CollisionMap* collision_map,
    const std::optional<drake::multibody::BodyIndex>&
        override_parent_body_index,
    const Parallelism parallelism) {
  DRAKE_THROW_UNLESS(collision_map != nullptr);

  const auto parent_body_index = override_parent_body_index.value_or(
      plant.GetBodyByName(collision_map->GetFrame()).index());

  FillVoxelGrid<voxelized_geometry_tools::CollisionCell>(
      plant, plant_context, geometries_to_ignore, CollisionCellFill,
      parent_body_index, parallelism, collision_map);
}

void FillTaggedObjectCollisionMap(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& plant_context,
    const std::unordered_set<drake::geometry::GeometryId>&
        geometries_to_ignore,
    voxelized_geometry_tools::TaggedObjectCollisionMap* collision_map,
    const std::optional<drake::multibody::BodyIndex>&
        override_parent_body_index,
    const Parallelism parallelism) {
  DRAKE_THROW_UNLESS(collision_map != nullptr);

  const auto parent_body_index = override_parent_body_index.value_or(
      plant.GetBodyByName(collision_map->GetFrame()).index());

  FillVoxelGrid<voxelized_geometry_tools::TaggedObjectCollisionCell>(
      plant, plant_context, geometries_to_ignore, TaggedObjectCollisionCellFill,
      parent_body_index, parallelism, collision_map);
}

voxelized_geometry_tools::CollisionCell CollisionCellFill(
    const DistanceQueryResult& distances) {
  const voxelized_geometry_tools::CollisionCell empty_cell(0.0f);
  const voxelized_geometry_tools::CollisionCell filled_cell(1.0f);
  if (distances.empty()) {
    return empty_cell;
  } else {
    return filled_cell;
  }
}

voxelized_geometry_tools::TaggedObjectCollisionCell
TaggedObjectCollisionCellFill(const DistanceQueryResult& distances) {
  const int64_t max_object_id =
      static_cast<int64_t>(std::numeric_limits<uint32_t>::max());
  const voxelized_geometry_tools::TaggedObjectCollisionCell empty_cell;
  if (distances.empty()) {
    return empty_cell;
  } else {
    // For now, only use the GeometryIds from the first penetration
    const auto& first_collision = distances.front();
    const int64_t object_id = first_collision.id_G.get_value();
    if (object_id <= max_object_id) {
      return voxelized_geometry_tools::TaggedObjectCollisionCell(
          1.0f, static_cast<uint32_t>(object_id));
    } else {
      throw std::runtime_error("object_id " + std::to_string(object_id) +
                                   " is larger than uint32_t");
    }
  }
}
}  // namespace planning
}  // namespace anzu
