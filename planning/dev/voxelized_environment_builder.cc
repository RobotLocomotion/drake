#include "drake/planning/dev/voxelized_environment_builder.h"

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
#include "drake/planning/dev/voxelized_environment_builder_internal.h"

namespace drake {
namespace planning {

using geometry::GeometryId;
using multibody::BodyIndex;
using multibody::MultibodyPlant;
using systems::Context;

using voxelized_geometry_tools::CollisionCell;
using voxelized_geometry_tools::CollisionMap;
using voxelized_geometry_tools::TaggedObjectCollisionCell;
using voxelized_geometry_tools::TaggedObjectCollisionMap;

namespace {

CollisionCell CollisionCellFill(
    const internal::DistanceQueryResult& distances) {
  const CollisionCell empty_cell(0.0f);
  const CollisionCell filled_cell(1.0f);
  if (distances.empty()) {
    return empty_cell;
  } else {
    return filled_cell;
  }
}

TaggedObjectCollisionCell TaggedObjectCollisionCellFill(
    const internal::DistanceQueryResult& distances) {
  const int64_t max_object_id =
      static_cast<int64_t>(std::numeric_limits<uint32_t>::max());
  const TaggedObjectCollisionCell empty_cell(0.0f);
  if (distances.empty()) {
    return empty_cell;
  } else {
    // For now, only use the GeometryIds from the first penetration
    const auto& first_collision = distances.front();
    const int64_t object_id = first_collision.id_G.get_value();
    if (object_id <= max_object_id) {
      return TaggedObjectCollisionCell(1.0f, static_cast<uint32_t>(object_id));
    } else {
      throw std::runtime_error(
          fmt::format("object_id {} is larger than uint32_t", object_id));
    }
  }
}

}  // namespace

CollisionMap BuildCollisionMap(
    const MultibodyPlant<double>& plant, const Context<double>& plant_context,
    const std::unordered_set<GeometryId>& geometries_to_ignore,
    const std::string& parent_body_name, const Eigen::Isometry3d& X_BG,
    const Eigen::Vector3d& grid_size, const double grid_resolution,
    const std::optional<BodyIndex>& override_parent_body_index,
    const Parallelism parallelism) {
  const common_robotics_utilities::voxel_grid::GridSizes grid_sizes(
      grid_resolution, grid_size.x(), grid_size.y(), grid_size.z());
  const CollisionCell empty_cell(0.0f);
  CollisionMap collision_map(X_BG, parent_body_name, grid_sizes, empty_cell);
  FillCollisionMap(plant, plant_context, geometries_to_ignore, &collision_map,
                   override_parent_body_index, parallelism);
  return collision_map;
}

TaggedObjectCollisionMap BuildTaggedObjectCollisionMap(
    const MultibodyPlant<double>& plant, const Context<double>& plant_context,
    const std::unordered_set<GeometryId>& geometries_to_ignore,
    const std::string& parent_body_name, const Eigen::Isometry3d& X_BG,
    const Eigen::Vector3d& grid_size, const double grid_resolution,
    const std::optional<BodyIndex>& override_parent_body_index,
    const Parallelism parallelism) {
  const common_robotics_utilities::voxel_grid::GridSizes grid_sizes(
      grid_resolution, grid_size.x(), grid_size.y(), grid_size.z());
  const TaggedObjectCollisionCell empty_cell;
  TaggedObjectCollisionMap collision_map(X_BG, parent_body_name, grid_sizes,
                                         empty_cell);
  FillTaggedObjectCollisionMap(plant, plant_context, geometries_to_ignore,
                               &collision_map, override_parent_body_index,
                               parallelism);
  return collision_map;
}

void FillCollisionMap(
    const MultibodyPlant<double>& plant, const Context<double>& plant_context,
    const std::unordered_set<GeometryId>& geometries_to_ignore,
    CollisionMap* collision_map,
    const std::optional<BodyIndex>& override_parent_body_index,
    const Parallelism parallelism) {
  DRAKE_THROW_UNLESS(collision_map != nullptr);

  const auto parent_body_index = override_parent_body_index.value_or(
      plant.GetBodyByName(collision_map->GetFrame()).index());

  internal::FillVoxelGrid<CollisionCell>(
      plant, plant_context, geometries_to_ignore, CollisionCellFill,
      parent_body_index, parallelism, collision_map);
}

void FillTaggedObjectCollisionMap(
    const MultibodyPlant<double>& plant, const Context<double>& plant_context,
    const std::unordered_set<GeometryId>& geometries_to_ignore,
    TaggedObjectCollisionMap* collision_map,
    const std::optional<BodyIndex>& override_parent_body_index,
    const Parallelism parallelism) {
  DRAKE_THROW_UNLESS(collision_map != nullptr);

  const auto parent_body_index = override_parent_body_index.value_or(
      plant.GetBodyByName(collision_map->GetFrame()).index());

  internal::FillVoxelGrid<TaggedObjectCollisionCell>(
      plant, plant_context, geometries_to_ignore, TaggedObjectCollisionCellFill,
      parent_body_index, parallelism, collision_map);
}

}  // namespace planning
}  // namespace drake
