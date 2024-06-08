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
#include <common_robotics_utilities/print.hpp>
#include <common_robotics_utilities/voxel_grid.hpp>
#include <voxelized_geometry_tools/collision_map.hpp>
#include <voxelized_geometry_tools/tagged_object_collision_map.hpp>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "planning/parallelism.h"

// Helper to print signed distance queries.
inline std::ostream& operator<<(
    std::ostream& strm,
    const drake::geometry::SignedDistanceToPoint<double>& distance) {
  strm << "SDP query " << distance.distance << " distance "
        << distance.grad_W.transpose() << " gradient "
        << distance.id_G << " geometry";
  return strm;
}

namespace anzu {
namespace planning {
/// The return value from a SignedDistanceToPoint collision query.
using DistanceQueryResult =
    std::vector<drake::geometry::SignedDistanceToPoint<double>>;

/// The type of the user-provided voxel-fill function.
template<typename T>
using VoxelFillFunction = std::function<T(const DistanceQueryResult&)>;

/// Builds a CollisionMap using the parameters specified.
/// CollisionMaps are a dense voxel grid where each cell is a
/// voxelized_geometry_tools::CollisionCell that stores P(occupancy) as a float.
/// CollisionMaps support computation of signed distance fields and
/// topological invariants (# of components, # of holes, # of voids).
/// Empty cells receive occupancy=0.0, filled cells receive occupancy=1.0.
/// @param plant MultibodyPlant model with a registered SceneGraph.
/// @param plant_context Context of plant.
/// @param geometries_to_ignore Set of geometries to ignore.
/// @param parent_body_name Name of parent body in the MultibodyPlant, used as
/// frame name in the constructed CollisionMap. If this name is not unique, or
/// does not correspond to an existing MbP body, use override_parent_body_index
/// to specfiy the parent body directly.
/// @param X_BG Pose of occupancy map frame G in frame of parent body B.
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
voxelized_geometry_tools::CollisionMap BuildCollisionMap(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& plant_context,
    const std::unordered_set<drake::geometry::GeometryId>&
        geometries_to_ignore,
    const std::string& parent_body_name, const Eigen::Isometry3d& X_BG,
    const Eigen::Vector3d& grid_size, const double grid_resolution,
    const std::optional<drake::multibody::BodyIndex>&
        override_parent_body_index = {},
    Parallelism parallelism = Parallelism::Max());

/// Builds a TaggedObjectCollisionMap using the parameters specified.
/// TaggedObjectCollisionMaps are a dense voxel grid where each cell is a
/// voxelized_geometry_tools::TaggedObjectCollisionCell that stores P(occupancy)
/// as a float and object_id as a uint32_t. TaggedObjectCollisionMaps support
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
/// frame name in the constructed CollisionMap. If this name is not unique, or
/// does not correspond to an existing MbP body, use override_parent_body_index
/// to specfiy the parent body directly.
/// @param X_BG Pose of occupancy map frame G in frame of parent body B.
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
voxelized_geometry_tools::TaggedObjectCollisionMap
BuildTaggedObjectCollisionMap(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& plant_context,
    const std::unordered_set<drake::geometry::GeometryId>&
        geometries_to_ignore,
    const std::string& parent_body_name, const Eigen::Isometry3d& X_BG,
    const Eigen::Vector3d& grid_size, const double grid_resolution,
    const std::optional<drake::multibody::BodyIndex>&
        override_parent_body_index = {},
    Parallelism parallelism = Parallelism::Max());

/// Fills an initialized VoxelGrid<T, Alloc, BackingStore> or derived class.
/// Cell fill is set by voxel_fill_fn.
/// @param plant MultibodyPlant model with a registered SceneGraph.
/// @param plant_context Context of plant.
/// @param geometries_to_ignore Set of geometries to ignore.
/// @param voxel_fill_fn Function to produce appropriate cell value.
/// Each voxel = voxel_fill_fn(query) where query is the result of the
/// collision check query.
/// @param parent_body_index Index of parent body in the MultibodyPlant.
/// @param grid VoxelGridBase<T>-derived class to fill.
template<typename T, typename BackingStore = std::vector<T>>
void FillVoxelGrid(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& plant_context,
    const std::unordered_set<drake::geometry::GeometryId>&
        geometries_to_ignore,
    const VoxelFillFunction<T>& voxel_fill_fn,
    drake::multibody::BodyIndex parent_body_index,
    Parallelism parallelism,
    common_robotics_utilities::voxel_grid
        ::VoxelGridBase<T, BackingStore>* grid) {
  DRAKE_THROW_UNLESS(grid != nullptr);
  DRAKE_THROW_UNLESS(grid->IsInitialized());
  DRAKE_THROW_UNLESS(grid->GetGridSizes().UniformCellSize());
  // Get the parent body of the voxel grid.
  const auto& grid_body = plant.get_body(parent_body_index);
  // Get pose of grid in world.
  const Eigen::Isometry3d X_WB =
      plant.EvalBodyPoseInWorld(plant_context, grid_body).GetAsIsometry3();
  const Eigen::Isometry3d& X_BGrid = grid->GetOriginTransform();
  const Eigen::Isometry3d X_WGrid = X_WB * X_BGrid;
  const Eigen::Isometry3d X_GridW = X_WGrid.inverse();
  // Note that cells are uniform in size.
  const double cell_size = grid->GetCellSizes().x();
  const double query_radius = cell_size;
  // The closest possible free space is 1/2 cell size away.
  const double min_check_radius = cell_size * 0.5;
  // The farthest colliding space is at most sqrt(3) * 1/2 cell size away, in
  // a far corner. Note sqrt(3) because the cells are uniform.
  const double max_check_radius = min_check_radius * std::sqrt(3.0);
  // 1e-6 is a sufficient check for all the uses in anzu, but this scaling
  // handles cases with very small cell sizes.
  const double boundary_check_threshold = std::min(cell_size * 1e-3, 1e-6);
  // Grab query object to test for collisions.
  const auto& query_object =
      plant.get_geometry_query_input_port().Eval<
          drake::geometry::QueryObject<double>>(plant_context);
  // Run an example query through to ensure the context is up-to-date.
  query_object.ComputeSignedDistanceToPoint(Eigen::Vector3d::Zero(), 0.01);
  // Freeze the context.
  plant_context.FreezeCache();
  // Iterate through the grid.
  DRAKE_OMP_PARALLEL_FOR_STATIC(parallelism)
  for (int64_t data_index = 0; data_index < grid->GetTotalCells();
       data_index++) {
    const auto grid_index = grid->DataIndexToGridIndex(data_index);
    const Eigen::Vector4d p_GridCo =
        grid->GridIndexToLocationInGridFrame(grid_index);
    const Eigen::Vector4d p_WCo = X_WGrid * p_GridCo;
    // Perform the point signed distance query.
    const DistanceQueryResult distances =
        query_object.ComputeSignedDistanceToPoint(
            p_WCo.head<3>(), query_radius);
    drake::log()->trace("Checking ({}) (x,y,z) {} Distances:\n{}", grid_index,
                        common_robotics_utilities::print::Print(p_WCo),
                        common_robotics_utilities::print::Print(distances));
    // Filter out distances that aren't inside the voxel.
    DistanceQueryResult colliding_distances;
    for (const auto& distance : distances) {
      // Filter out distances to geometries we want to ignore.
      if (geometries_to_ignore.count(distance.id_G) == 0) {
        if (distance.distance < min_check_radius) {
          // If the distance is within min check radius, we know it is inside
          // the current cell.
          colliding_distances.push_back(distance);
        } else if (distance.distance < max_check_radius) {
          // If the distance is within the max check radius, it might be
          // inside the current cell. We check if the reported nearest point,
          // p_GN, is within the current voxel. To avoid discretization
          // artifacts, a very small buffer boundary_check_threshold is used so
          // that points on the boundary of the current voxel are not recorded
          // as inside the voxel.
          const Eigen::Isometry3d X_WGeom =
              query_object.GetPoseInWorld(distance.id_G).GetAsIsometry3();
          const Eigen::Vector3d p_GridN = X_GridW * X_WGeom * distance.p_GN;
          const double max_axis_dist =
              (p_GridN - p_GridCo.head<3>()).lpNorm<Eigen::Infinity>();
          if (max_axis_dist < (min_check_radius - boundary_check_threshold)) {
            colliding_distances.push_back(distance);
          }
        }
      }
    }
    // Fill the voxel.
    grid->GetDataIndexMutable(data_index) = voxel_fill_fn(colliding_distances);
  }
  // Thaw the context.
  plant_context.UnfreezeCache();
}

/// Fills an initialized CollisionMapGrid.
/// Empty cells receive occupancy=0.0, filled cells receive occupancy=1.0.
/// @param plant MultibodyPlant model with a registered SceneGraph.
/// @param plant_context Context of plant.
/// @param geometries_to_ignore Set of geometries to ignore.
/// @param collision_map TaggedObjectCollisionMapGrid to fill.
/// @param override_parent_body_index Optionally provide a body index to
/// override using the collision map frame name to identify the parent body. Use
/// this if the frame name is not unique, or if the frame name does not match an
/// existing MbP body (e.g. the name is a TF-compatible name incompatible with
/// GetBodyByName).
void FillCollisionMap(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& plant_context,
    const std::unordered_set<drake::geometry::GeometryId>&
        geometries_to_ignore,
    voxelized_geometry_tools::CollisionMap* collision_map,
    const std::optional<drake::multibody::BodyIndex>&
        override_parent_body_index = {},
    Parallelism parallelism = Parallelism::Max());

/// Fills an initialized TaggedObjectCollisionMapGrid.
/// Empty cells receive occupancy=0.0, filled cells receive occupancy=1.0.
/// Filled cells of the environment receive object_id values corresponding to
/// the integer values of the GeometryId in that location. If your environment
/// has more than 2^32 GeometryIds, this will throw.
/// @param plant MultibodyPlant model with a registered SceneGraph.
/// @param plant_context Context of plant.
/// @param geometries_to_ignore Set of geometries to ignore.
/// @param collision_map TaggedObjectCollisionMapGrid to fill.
/// @param override_parent_body_index Optionally provide a body index to
/// override using the collision map frame name to identify the parent body. Use
/// this if the frame name is not unique, or if the frame name does not match an
/// existing MbP body (e.g. the name is a TF-compatible name incompatible with
/// GetBodyByName).
void FillTaggedObjectCollisionMap(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& plant_context,
    const std::unordered_set<drake::geometry::GeometryId>&
        geometries_to_ignore,
    voxelized_geometry_tools::TaggedObjectCollisionMap* collision_map,
    const std::optional<drake::multibody::BodyIndex>&
        override_parent_body_index = {},
    Parallelism parallelism = Parallelism::Max());

/// Returns the voxelized_geometry_tools::CollisionCell that corresponds to the
/// query. If query is empty, it returns a cell with occupancy=0.0; otherwise it
/// returns a cell with occupancy=1.0.
/// @param query Result from calling ComputeSignedDistanceToPoint(), filtered
/// to only contain returns inside the current cell.
voxelized_geometry_tools::CollisionCell CollisionCellFill(
    const DistanceQueryResult& distances);

/// Returns the voxelized_geometry_tools::TaggedObjectCollisionCell that
/// corresponds to the query. If query is empty, it returns a cell with
/// occupancy=0.0;/ otherwise it returns a cell with occupancy=1.0 and object_id
/// value corresponding to the integer values of the GeometryId of the colliding
/// object in the first SignedDistanceToPoint returned.
/// @param query Result from calling ComputeSignedDistanceToPoint(), filtered
/// to only contain returns inside the current cell.
voxelized_geometry_tools::TaggedObjectCollisionCell
TaggedObjectCollisionCellFill(const DistanceQueryResult& distances);
}  // namespace planning
}  // namespace anzu
