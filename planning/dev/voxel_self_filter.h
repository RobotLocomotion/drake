#pragma once

#include <optional>

#include <Eigen/Geometry>

#include "drake/common/parallelism.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/dev/sphere_robot_model_collision_checker.h"
#include "drake/planning/dev/voxel_occupancy_map.h"
#include "drake/planning/dev/voxel_tagged_object_occupancy_map.h"

namespace drake {
namespace planning {

/// Self-filter implementation for VoxelOccupancyMap environments.
/// Self-filter marks voxels belonging to the robot as empty so that they do not
/// produce false collisions in a voxelized environment used for collision
/// checking.
/// @param collision_checker Sphere-model collision checker that provides the
/// sphere model of robot geometry and performs forward kinematics.
/// @param q Current configuration of the robot.
/// @param padding Padding to inflate the spheres of the collision model to use
/// in the self-filter. @pre >= 0.0.
/// @param occupancy_map Current environment. @pre != nullptr.
/// @param parallelism Parallelism to use.
/// @param context_number Optional context number for use in parallel contexts.
void SelfFilter(const SphereRobotModelCollisionChecker& collision_checker,
                const Eigen::VectorXd& q, double padding,
                multibody::BodyIndex grid_body_index,
                VoxelOccupancyMap* occupancy_map, Parallelism parallelism,
                std::optional<int> context_number = std::nullopt);

/// Self-filter implementation for VoxelTaggedObjectOccupancyMap environments.
/// Self-filter marks voxels belonging to the robot as empty so that they do not
/// produce false collisions in a voxelized environment used for collision
/// checking.
/// @param collision_checker Sphere-model collision checker that provides the
/// sphere model of robot geometry and performs forward kinematics.
/// @param q Current configuration of the robot.
/// @param padding Padding to inflate the spheres of the collision model to use
/// in the self-filter. @pre >= 0.0.
/// @param occupancy_map Current environment. @pre != nullptr.
/// @param parallelism Parallelism to use.
/// @param context_number Optional context number for use in parallel contexts.
void SelfFilter(const SphereRobotModelCollisionChecker& collision_checker,
                const Eigen::VectorXd& q, double padding,
                multibody::BodyIndex grid_body_index,
                VoxelTaggedObjectOccupancyMap* occupancy_map,
                Parallelism parallelism,
                std::optional<int> context_number = std::nullopt);

}  // namespace planning
}  // namespace drake
