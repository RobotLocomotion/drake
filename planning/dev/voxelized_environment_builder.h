#pragma once

#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>

#include "drake/common/parallelism.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/dev/voxel_occupancy_map.h"
#include "drake/planning/dev/voxel_tagged_object_occupancy_map.h"

namespace drake {
namespace planning {

/// Builds a VoxelOccupancyMap using the parameters specified.
/// VoxelOccupancyMaps are a dense voxel grid where each cell is a
/// voxelized_geometry_tools::OccupancyCell that stores P(occupancy) as a float.
/// OccupancyMaps support computation of signed distance fields and
/// topological invariants (# of components, # of holes, # of voids).
/// Empty cells receive occupancy=0.0, filled cells receive occupancy=1.0.
/// @param plant MultibodyPlant model with a registered SceneGraph.
/// @param plant_context Context of plant.
/// @param geometries_to_ignore Set of geometries to ignore.
/// @param parent_body_name Name of parent body in the MultibodyPlant, used as
/// frame name in the constructed OccupancyMap. If this name is not unique, or
/// does not correspond to an existing MbP body, use override_parent_body_index
/// to specfiy the parent body directly.
/// @param X_PG Pose of occupancy map frame G in frame of parent body P.
/// @param grid_size Size of occupancy map in meters. If you specify a
/// grid_size that is not evenly divisible by grid_resolution, you will get a
/// larger grid with num_cells = ceil(size/resolution).
/// @param grid_resolution Cell size (in meters) for all Voxel grids used by
/// the builder. All grids must have uniform cell sizes.
/// @pre grid_resolution > 0
/// @param override_parent_body_index Optionally provide a body index to
/// override using parent_body_name to identify the parent body. Use this if the
/// parent body name is not unique, or if the frame name does not match an
/// existing MbP body (e.g. the name is a TF-compatible name incompatible with
/// GetBodyByName).
VoxelOccupancyMap BuildOccupancyMap(
    const multibody::MultibodyPlant<double>& plant,
    const systems::Context<double>& plant_context,
    const std::unordered_set<geometry::GeometryId>& geometries_to_ignore,
    const std::string& parent_body_name, const math::RigidTransformd& X_PG,
    const Eigen::Vector3d& grid_dimensions, const double grid_resolution,
    const std::optional<multibody::BodyIndex>& override_parent_body_index = {},
    Parallelism parallelism = Parallelism::Max());

/// Builds a VoxelTaggedObjectOccupancyMap using the parameters specified.
/// VoxelTaggedObjectOccupancyMaps are a dense voxel grid where each cell is a
/// voxelized_geometry_tools::TaggedObjectOccupancyCell that stores P(occupancy)
/// as a float and object_id as a uint32_t. TaggedObjectOccupancyMaps support
/// computation of signed distance fields and the first three topological
/// invariants (# of components, # of holes, # of voids) as well as a limited
/// form of spatial partioning.
/// Empty cells receive occupancy=0.0, filled cells receive occupancy=1.0.
/// Filled cells of the environment receive object_id values corresponding to
/// the integer values of the GeometryId in that location. If your environment
/// has more than 2^32 GeometryIds, this will throw.
/// @param plant MultibodyPlant model with a registered SceneGraph.
/// @param plant_context Context of plant.
/// @param geometries_to_ignore Set of geometries to ignore.
/// @param parent_body_name Name of parent body in the MultibodyPlant, used as
/// frame name in the constructed OccupancyMap. If this name is not unique, or
/// does not correspond to an existing MbP body, use override_parent_body_index
/// to specfiy the parent body directly.
/// @param X_PG Pose of occupancy map frame G in frame of parent body P.
/// @param grid_dimensions Size of occupancy map in meters. If you specify a
/// grid_size that is not evenly divisible by grid_resolution, you will get a
/// larger grid with num_cells = ceil(size/resolution).
/// @param grid_resolution Cell size (in meters) for all Voxel grids used by
/// the builder. All grids must have uniform cell sizes.
/// @pre grid_resolution > 0
/// @param override_parent_body_index Optionally provide a body index to
/// override using parent_body_name to identify the parent body. Use this if the
/// parent body name is not unique, or if the frame name does not match an
/// existing MbP body (e.g. the name is a TF-compatible name incompatible with
/// GetBodyByName).
VoxelTaggedObjectOccupancyMap BuildTaggedObjectOccupancyMap(
    const multibody::MultibodyPlant<double>& plant,
    const systems::Context<double>& plant_context,
    const std::unordered_set<geometry::GeometryId>& geometries_to_ignore,
    const std::string& parent_body_name, const math::RigidTransformd& X_PG,
    const Eigen::Vector3d& grid_dimensions, const double grid_resolution,
    const std::optional<multibody::BodyIndex>& override_parent_body_index = {},
    Parallelism parallelism = Parallelism::Max());

/// Fills an initialized VoxelOccupancyMap.
/// Empty cells receive occupancy=0.0, filled cells receive occupancy=1.0.
/// @param plant MultibodyPlant model with a registered SceneGraph.
/// @param plant_context Context of plant.
/// @param geometries_to_ignore Set of geometries to ignore.
/// @param occupancy_map TaggedObjectOccupancyMapGrid to fill.
/// @param override_parent_body_index Optionally provide a body index to
/// override using the collision map frame name to identify the parent body. Use
/// this if the frame name is not unique, or if the frame name does not match an
/// existing MbP body (e.g. the name is a TF-compatible name incompatible with
/// GetBodyByName).
void FillOccupancyMap(
    const multibody::MultibodyPlant<double>& plant,
    const systems::Context<double>& plant_context,
    const std::unordered_set<geometry::GeometryId>& geometries_to_ignore,
    VoxelOccupancyMap* occupancy_map,
    const std::optional<multibody::BodyIndex>& override_parent_body_index = {},
    Parallelism parallelism = Parallelism::Max());

/// Fills an initialized VoxelTaggedObjectOccupancyMap.
/// Empty cells receive occupancy=0.0, filled cells receive occupancy=1.0.
/// Filled cells of the environment receive object_id values corresponding to
/// the integer values of the GeometryId in that location. If your environment
/// has more than 2^32 GeometryIds, this will throw.
/// @param plant MultibodyPlant model with a registered SceneGraph.
/// @param plant_context Context of plant.
/// @param geometries_to_ignore Set of geometries to ignore.
/// @param occupancy_map TaggedObjectOccupancyMapGrid to fill.
/// @param override_parent_body_index Optionally provide a body index to
/// override using the collision map frame name to identify the parent body. Use
/// this if the frame name is not unique, or if the frame name does not match an
/// existing MbP body (e.g. the name is a TF-compatible name incompatible with
/// GetBodyByName).
void FillTaggedObjectOccupancyMap(
    const multibody::MultibodyPlant<double>& plant,
    const systems::Context<double>& plant_context,
    const std::unordered_set<geometry::GeometryId>& geometries_to_ignore,
    VoxelTaggedObjectOccupancyMap* occupancy_map,
    const std::optional<multibody::BodyIndex>& override_parent_body_index = {},
    Parallelism parallelism = Parallelism::Max());

}  // namespace planning
}  // namespace drake
