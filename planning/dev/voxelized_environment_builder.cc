#include "drake/planning/dev/voxelized_environment_builder.h"

#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <voxelized_geometry_tools/occupancy_map.hpp>
#include <voxelized_geometry_tools/tagged_object_occupancy_map.hpp>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/dev/voxel_occupancy_map_internal.h"
#include "drake/planning/dev/voxel_tagged_object_occupancy_map_internal.h"
#include "drake/planning/dev/voxelized_environment_builder_internal.h"

namespace drake {
namespace planning {

using geometry::GeometryId;
using multibody::BodyIndex;
using multibody::MultibodyPlant;
using systems::Context;

using voxelized_geometry_tools::OccupancyCell;
using voxelized_geometry_tools::TaggedObjectOccupancyCell;

namespace {

OccupancyCell OccupancyCellFill(
    const internal::DistanceQueryResult& distances) {
  const OccupancyCell empty_cell(0.0f);
  const OccupancyCell filled_cell(1.0f);
  if (distances.empty()) {
    return empty_cell;
  } else {
    return filled_cell;
  }
}

TaggedObjectOccupancyCell TaggedObjectOccupancyCellFill(
    const internal::DistanceQueryResult& distances) {
  const int64_t max_object_id =
      static_cast<int64_t>(std::numeric_limits<uint32_t>::max());
  const TaggedObjectOccupancyCell empty_cell(0.0f, 0u);
  if (distances.empty()) {
    return empty_cell;
  } else {
    // For now, only use the GeometryIds from the first penetration
    const auto& first_collision = distances.front();
    const int64_t object_id = first_collision.id_G.get_value();
    if (object_id <= max_object_id) {
      return TaggedObjectOccupancyCell(1.0f, static_cast<uint32_t>(object_id));
    } else {
      throw std::runtime_error(
          fmt::format("object_id {} is larger than uint32_t", object_id));
    }
  }
}

}  // namespace

VoxelOccupancyMap BuildOccupancyMap(
    const MultibodyPlant<double>& plant, const Context<double>& plant_context,
    const std::unordered_set<GeometryId>& geometries_to_ignore,
    const std::string& parent_body_name, const math::RigidTransformd& X_PG,
    const Eigen::Vector3d& grid_dimensions, const double grid_resolution,
    const std::optional<BodyIndex>& override_parent_body_index,
    const Parallelism parallelism) {
  VoxelOccupancyMap occupancy_map(parent_body_name, X_PG, grid_dimensions,
                                  grid_resolution, 0.0f);
  FillOccupancyMap(plant, plant_context, geometries_to_ignore, &occupancy_map,
                   override_parent_body_index, parallelism);
  return occupancy_map;
}

VoxelTaggedObjectOccupancyMap BuildTaggedObjectOccupancyMap(
    const MultibodyPlant<double>& plant, const Context<double>& plant_context,
    const std::unordered_set<GeometryId>& geometries_to_ignore,
    const std::string& parent_body_name, const math::RigidTransformd& X_PG,
    const Eigen::Vector3d& grid_dimensions, const double grid_resolution,
    const std::optional<BodyIndex>& override_parent_body_index,
    const Parallelism parallelism) {
  VoxelTaggedObjectOccupancyMap occupancy_map(
      parent_body_name, X_PG, grid_dimensions, grid_resolution, 0.0f, 0u);
  FillTaggedObjectOccupancyMap(plant, plant_context, geometries_to_ignore,
                               &occupancy_map, override_parent_body_index,
                               parallelism);
  return occupancy_map;
}

void FillOccupancyMap(
    const MultibodyPlant<double>& plant, const Context<double>& plant_context,
    const std::unordered_set<GeometryId>& geometries_to_ignore,
    VoxelOccupancyMap* occupancy_map,
    const std::optional<BodyIndex>& override_parent_body_index,
    const Parallelism parallelism) {
  DRAKE_THROW_UNLESS(occupancy_map != nullptr);
  auto& internal_occupancy_map =
      internal::GetMutableInternalOccupancyMap(*occupancy_map);

  const auto parent_body_index = override_parent_body_index.value_or(
      plant.GetBodyByName(occupancy_map->parent_body_name()).index());

  internal::FillVoxelGrid<OccupancyCell>(
      plant, plant_context, geometries_to_ignore, OccupancyCellFill,
      parent_body_index, parallelism, &internal_occupancy_map);
}

void FillTaggedObjectOccupancyMap(
    const MultibodyPlant<double>& plant, const Context<double>& plant_context,
    const std::unordered_set<GeometryId>& geometries_to_ignore,
    VoxelTaggedObjectOccupancyMap* occupancy_map,
    const std::optional<BodyIndex>& override_parent_body_index,
    const Parallelism parallelism) {
  DRAKE_THROW_UNLESS(occupancy_map != nullptr);
  auto& internal_occupancy_map =
      internal::GetMutableInternalTaggedObjectOccupancyMap(*occupancy_map);

  const auto parent_body_index = override_parent_body_index.value_or(
      plant.GetBodyByName(occupancy_map->parent_body_name()).index());

  internal::FillVoxelGrid<TaggedObjectOccupancyCell>(
      plant, plant_context, geometries_to_ignore, TaggedObjectOccupancyCellFill,
      parent_body_index, parallelism, &internal_occupancy_map);
}

}  // namespace planning
}  // namespace drake
