#pragma once

#include <chrono>
#include <functional>
#include <optional>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/parallelism.hpp>
#include <common_robotics_utilities/voxel_grid.hpp>

#include "drake/common/drake_assert.h"
#include "drake/common/parallelism.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/dev/sphere_robot_model_collision_checker.h"

// Our linter rejects logging from header files, but this isn't really a header
// file. It is listed as `srcs` not `hdrs` in the BUILD file and is more like an
// `*.inc` file than a true header.
#include "drake/common/text_logging.h"  // drakelint: ignore

namespace drake {
namespace planning {
namespace internal {

/// Helper for SelfFilter, sets the voxel cells corresponding to the provided
/// sphere.
/// @param environment Voxelized environment.
/// @param sphere Sphere in the body frame of the voxelized environment.
/// @param padding Padding to inflate the sphere. @pre >= 0.0.
/// @param cell_value Value to set corresponding cells.
template <typename T, typename BackingStore = std::vector<T>>
void SetSphereCells(
    common_robotics_utilities::voxel_grid::VoxelGridBase<T, BackingStore>* const
        environment,
    const SphereSpecification& sphere, double padding, const T& cell_value) {
  DRAKE_THROW_UNLESS(environment != nullptr);
  DRAKE_THROW_UNLESS(environment->IsInitialized());
  DRAKE_THROW_UNLESS(environment->HasUniformCellSize());
  DRAKE_THROW_UNLESS(padding >= 0.0);

  // Add check buffer equal to the cell center->cell corner distance.
  const double cell_size = environment->GetCellSizes().x();
  const double center_to_corner = cell_size * 0.5 * std::sqrt(3.0);

  const Eigen::Vector4d& p_BSo = sphere.Origin();
  const double bound = sphere.Radius() + padding;
  const double threshold = bound + center_to_corner;
  const double squared_threshold = std::pow(threshold, 2.0);

  const Eigen::Vector4d corner_offset(bound, bound, bound, 0.0);
  const Eigen::Vector4d min_corner = p_BSo - corner_offset;
  const Eigen::Vector4d max_corner = p_BSo + corner_offset;
  const auto min_corner_index = environment->LocationToGridIndex4d(min_corner);
  const auto max_corner_index = environment->LocationToGridIndex4d(max_corner);

  for (int64_t x_index = min_corner_index.X(); x_index <= max_corner_index.X();
       ++x_index) {
    for (int64_t y_index = min_corner_index.Y();
         y_index <= max_corner_index.Y(); ++y_index) {
      for (int64_t z_index = min_corner_index.Z();
           z_index <= max_corner_index.Z(); ++z_index) {
        const common_robotics_utilities::voxel_grid::GridIndex query_index(
            x_index, y_index, z_index);

        if (environment->IndexInBounds(query_index)) {
          // Compute the position of the center of the voxel.
          const Eigen::Vector4d p_BCo =
              environment->GridIndexToLocation(query_index);

          const double squared_distance = (p_BSo - p_BCo).squaredNorm();
          if (squared_distance <= squared_threshold) {
            environment->SetIndex(query_index, cell_value);
          }
        }
      }
    }
  }
}

/// Self-filter implementation for generic voxelized environments.
/// Self-filter marks voxels belonging to the robot as empty so that they do not
/// produce false collisions in a voxelized environment used for collision
/// checking.
/// @param collision_checker Sphere-model collision checker that provides the
/// sphere model of robot geometry and performs forward kinematics.
/// @param q Current configuration of the robot.
/// @param padding Padding to inflate the spheres of the collision model to use
/// in the self-filter. @pre >= 0.0.
/// @param environment Current environment. @pre != nullptr.
/// @param empty_cell_value Value for empty cells.
/// @param parallelism Parallelism to use.
/// @param context_number Optional context number for use in parallel contexts.
template <typename T, typename BackingStore = std::vector<T>>
void SelfFilter(
    const SphereRobotModelCollisionChecker& collision_checker,
    const Eigen::VectorXd& q, double padding,
    multibody::BodyIndex grid_body_index,
    common_robotics_utilities::voxel_grid::VoxelGridBase<T, BackingStore>* const
        environment,
    const T& empty_cell_value, Parallelism parallelism,
    std::optional<int> context_number = std::nullopt) {
  DRAKE_THROW_UNLESS(padding >= 0.0);
  DRAKE_THROW_UNLESS(environment != nullptr);
  DRAKE_THROW_UNLESS(environment->IsInitialized());
  DRAKE_THROW_UNLESS(environment->HasUniformCellSize());

  const auto start_time = std::chrono::steady_clock::now();

  // Get the self-filter spheres.
  const std::vector<Eigen::Isometry3d> X_WB_set =
      collision_checker.ComputeBodyPoses(q, context_number);
  const std::vector<BodySpheres> spheres_in_world_frame =
      collision_checker.ComputeSphereLocationsInWorldFrame(X_WB_set);

  // Get the pose of the grid body in world.
  const Eigen::Isometry3d X_BW = X_WB_set.at(grid_body_index).inverse();

  // Transform the self-filter spheres into grid body frame.
  std::vector<SphereSpecification> filter_spheres;
  for (const auto& body_spheres : spheres_in_world_frame) {
    for (const auto& [sphere_id, sphere] : body_spheres) {
      unused(sphere_id);
      const Eigen::Vector4d& p_WSo = sphere.Origin();
      const Eigen::Vector4d p_BSo = X_BW * p_WSo;
      const SphereSpecification filter_sphere(p_BSo, sphere.Radius());
      filter_spheres.push_back(filter_sphere);
    }
  }

  // Loop through filter spheres.
  const auto per_sphere_work = [&](const int, const int64_t sphere_num) {
    SetSphereCells(environment, filter_spheres.at(sphere_num), padding,
                   empty_cell_value);
  };

  drake::log()->debug("SelfFilter uses {} thread(s)",
                      parallelism.num_threads());

  common_robotics_utilities::parallelism::StaticParallelForIndexLoop(
      common_robotics_utilities::parallelism::DegreeOfParallelism(
          parallelism.num_threads()),
      0, filter_spheres.size(), per_sphere_work,
      common_robotics_utilities::parallelism::ParallelForBackend::
          BEST_AVAILABLE);

  const auto end_time = std::chrono::steady_clock::now();
  drake::log()->debug(
      "Self-filter took {} seconds",
      std::chrono::duration<double>(end_time - start_time).count());
}

}  // namespace internal
}  // namespace planning
}  // namespace drake
