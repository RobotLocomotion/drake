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
#include <common_robotics_utilities/parallelism.hpp>
#include <common_robotics_utilities/voxel_grid.hpp>
#include <fmt/format.h>
#include <voxelized_geometry_tools/occupancy_map.hpp>
#include <voxelized_geometry_tools/tagged_object_occupancy_map.hpp>

#include "drake/common/parallelism.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"

// Our linter rejects logging from header files, but this isn't really a header
// file. It is listed as `srcs` not `hdrs` in the BUILD file and is more like an
// `*.inc` file than a true header.
#include "drake/common/text_logging.h"  // drakelint: ignore

namespace drake {
namespace planning {
namespace internal {
/// The return value from a SignedDistanceToPoint collision query.
using DistanceQueryResult =
    std::vector<drake::geometry::SignedDistanceToPoint<double>>;

/// The type of the user-provided voxel-fill function.
template <typename T>
using VoxelFillFunction = std::function<T(const DistanceQueryResult&)>;

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
template <typename T, typename BackingStore = std::vector<T>>
void FillVoxelGrid(
    const multibody::MultibodyPlant<double>& plant,
    const systems::Context<double>& plant_context,
    const std::unordered_set<geometry::GeometryId>& geometries_to_ignore,
    const VoxelFillFunction<T>& voxel_fill_fn,
    multibody::BodyIndex parent_body_index, Parallelism parallelism,
    common_robotics_utilities::voxel_grid::VoxelGridBase<T, BackingStore>*
        grid) {
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
      plant.get_geometry_query_input_port()
          .Eval<drake::geometry::QueryObject<double>>(plant_context);

  // Run an example query through to ensure the context is up-to-date.
  query_object.ComputeSignedDistanceToPoint(Eigen::Vector3d::Zero(), 0.01);

  // Freeze the context.
  plant_context.FreezeCache();

  // Iterate through the grid.
  const auto per_voxel_work = [&](const int, const int64_t data_index) {
    const auto grid_index = grid->DataIndexToGridIndex(data_index);
    const Eigen::Vector4d p_GridCo =
        grid->GridIndexToLocationInGridFrame(grid_index);
    const Eigen::Vector4d p_WCo = X_WGrid * p_GridCo;

    // Perform the point signed distance query.
    const DistanceQueryResult distances =
        query_object.ComputeSignedDistanceToPoint(p_WCo.head<3>(),
                                                  query_radius);

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
  };

  drake::log()->debug("FillVoxelGrid uses {} thread(s)",
                      parallelism.num_threads());

  common_robotics_utilities::parallelism::StaticParallelForIndexLoop(
      common_robotics_utilities::parallelism::DegreeOfParallelism(
          parallelism.num_threads()),
      0, grid->GetTotalCells(), per_voxel_work,
      common_robotics_utilities::parallelism::ParallelForBackend::
          BEST_AVAILABLE);

  // Thaw the context.
  plant_context.UnfreezeCache();
}

}  // namespace internal
}  // namespace planning
}  // namespace drake
