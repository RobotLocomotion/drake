#pragma once

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <voxelized_geometry_tools/collision_map.hpp>
#include <voxelized_geometry_tools/signed_distance_field.hpp>
#include <voxelized_geometry_tools/tagged_object_collision_map.hpp>

#include "drake/planning/collision_checker.h"
#include "drake/planning/collision_checker_params.h"
#include "drake/planning/robot_diagram.h"
#include "drake/planning/sphere_robot_model_collision_checker.h"

namespace drake {
namespace planning {
using EnvironmentSDF = voxelized_geometry_tools::SignedDistanceField<float>;

// Typedef for SDF pointer that can be shared between multiple collision
// checkers.
using EnvironmentSDFConstSharedPtr = std::shared_ptr<const EnvironmentSDF>;

inline EnvironmentSDFConstSharedPtr MakeConstSharedPtrCopy(
    const EnvironmentSDF& sdf) {
  return EnvironmentSDFConstSharedPtr(
      reinterpret_cast<const EnvironmentSDF*>(sdf.Clone().release()));
}

/// Collision checker using a voxelized environment model.
class VoxelizedEnvironmentCollisionChecker final
    : public SphereRobotModelCollisionChecker {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  // N.B. The copy constructor is protected for use in implementing Clone().
  void operator=(const VoxelizedEnvironmentCollisionChecker&) = delete;
  /** @} */

  /// Construct a collision checker for voxelized environments.
  /// @param model a Diagram+MbP+SG model of the robot and environment.
  /// @param robot_model_instances is a vector of model instance indices that
  /// identify which model instances belong to the robot.
  /// @param distance_fn Configuration (probably weighted) distance function.
  /// @param edge_step_size Step size for collision checking, in radians.
  /// Collision checking of edges q1->q2 is performed by interpolating from q1
  /// to q2 at edge_step_size steps and checking the interpolated
  /// configuration for collision.
  /// @param env_collision_padding Additional padding to apply to all
  /// robot-environment collision queries. If distance between robot and
  /// environment is less than padding, the checker reports a collision.
  /// @param self_collision_padding Additional padding to apply to all
  /// robot-robot self collision queries. If distance between robot and
  /// itself is less than padding, the checker reports a collision.
  explicit VoxelizedEnvironmentCollisionChecker(
      drake::planning::CollisionCheckerParams params);

  /// Update the voxelized environment.
  /// @param environment_name Name of the environment model to update. If the
  /// name is already in use, the new model replaces the old. To remove a model,
  /// provide a default-constructed CollisionMapGrid that is not initialized.
  /// @param environment Voxelized environment model.
  /// @param override_environment_body_index Optionally provide a body index to
  /// override the environment frame name -> body lookup. Use this if the frame
  /// name is not unique, or if the frame name does not match an existing MbP
  /// body.
  void UpdateEnvironment(
      const std::string& environment_name,
      const voxelized_geometry_tools::CollisionMap& environment,
      const std::optional<drake::multibody::BodyIndex>&
          override_environment_body_index = {});

  /// Update the voxelized environment.
  /// @param environment_name Name of the environment model to update. If the
  /// name is already in use, the new model replaces the old. To remove a model,
  /// provide a default-constructed TaggedObjectCollisionMapGrid that is not
  /// initialized.
  /// @param environment Voxelized environment model.
  /// @param override_environment_body_index Optionally provide a body index to
  /// override the environment frame name -> body lookup. Use this if the frame
  /// name is not unique, or if the frame name does not match an existing MbP
  /// body.
  void UpdateEnvironment(
      const std::string& environment_name,
      const voxelized_geometry_tools::TaggedObjectCollisionMap& environment,
      const std::optional<drake::multibody::BodyIndex>&
          override_environment_body_index = {});

  /// Update the voxelized environment.
  /// @param environment_name Name of the environment model to update. If the
  /// name is already in use, the new model replaces the old. To remove a model,
  /// provide a default-constructed SignedDistanceField that is not initialized.
  /// @param environment_sdf signed distance field of voxelized environment.
  /// @param override_environment_body_index Optionally provide a body index to
  /// override the environment frame name -> body lookup. Use this if the frame
  /// name is not unique, or if the frame name does not match an existing MbP
  /// body.
  void UpdateEnvironment(
      const std::string& environment_name,
      const EnvironmentSDF& environment_sdf,
      const std::optional<drake::multibody::BodyIndex>&
          override_environment_body_index = {});

  /// Update the voxelized environment.
  /// @param environment_name Name of the environment model to update. If the
  /// name is already in use, the new model replaces the old. To remove a model,
  /// provide a default-constructed SignedDistanceField that is not initialized.
  /// @param environment_sdf signed distance field of voxelized environment.
  /// @param override_environment_body_index Optionally provide a body index to
  /// override the environment frame name -> body lookup. Use this if the frame
  /// name is not unique, or if the frame name does not match an existing MbP
  /// body.
  void UpdateEnvironment(
      const std::string& environment_name,
      const EnvironmentSDFConstSharedPtr& environment_sdf,
      const std::optional<drake::multibody::BodyIndex>&
          override_environment_body_index = {});

  /// Remove the voxelized model corresponding to `environment_name`.
  bool RemoveEnvironment(const std::string& environment_name);

  const std::map<std::string, EnvironmentSDFConstSharedPtr>&
  EnvironmentSDFs() const { return environment_sdfs_; }

  const std::map<std::string, drake::multibody::BodyIndex>&
  EnvironmentSDFBodies() const { return environment_sdf_bodies_; }

  /// Query the (distance, gradient) of the provided point from obstacles.
  /// @param context Context of the MbP model. Unused.
  /// @param p_WQ Query position in world frame W.
  /// @param query_radius Gradients do not need to be computed for queries
  /// with distance > query_radius. This parameter is needed because the
  /// default implementation calls ComputePointSignedDistanceAndGradient, and
  /// only needing to check within a bound can improve performance.
  /// @param X_WB_set Poses X_WB for all bodies in the model. This is
  /// used to move gradients from signed distance fields back to world frame.
  /// @param X_WB_inverse_set Poses X_BW for all bodies in the model. This is
  /// used to move the provided p_WQ into the frame of signed distance fields.
  /// @return signed distances and gradients, where signed distance is positive
  /// if @param p_WQ is outside of objects, and negative if it is inside. The
  /// gradient is ∂d/∂p.
  PointSignedDistanceAndGradientResult
  ComputePointToEnvironmentSignedDistanceAndGradient(
      const drake::systems::Context<double>& plant_context,
      const drake::geometry::QueryObject<double>& query_object,
      const Eigen::Vector4d& p_WQ, double query_radius,
      const std::vector<Eigen::Isometry3d>& X_WB_set,
      const std::vector<Eigen::Isometry3d>& X_WB_inverse_set) const override;

  /// Query the distance of the provided point from obstacles.
  /// @param context Context of the MbP model. Unused.
  /// @param p_WQ Query position in world frame W.
  /// @param query_radius Gradients do not need to be computed for queries
  /// with distance > query_radius. This parameter is needed because the
  /// default implementation calls ComputePointSignedDistanceAndGradient, and
  /// only needing to check within a bound can improve performance.
  /// @param X_WB_set Poses X_WB for all bodies in the model. This is
  /// used to move gradients from signed distance fields back to world frame.
  /// @param X_WB_inverse_set Poses X_BW for all bodies in the model. This is
  /// used to move the provided p_WQ into the frame of signed distance fields.
  /// @return signed distances where signed distance is positive
  /// if @param p_WQ is outside of objects, and negative if it is inside.
  PointSignedDistanceAndGradientResult ComputePointToEnvironmentSignedDistance(
      const drake::systems::Context<double>& plant_context,
      const drake::geometry::QueryObject<double>& query_object,
      const Eigen::Vector4d& p_WQ, double query_radius,
      const std::vector<Eigen::Isometry3d>& X_WB_set,
      const std::vector<Eigen::Isometry3d>& X_WB_inverse_set) const override;

 protected:
  /// To support Clone(), allow copying (but not move nor assign).
  explicit VoxelizedEnvironmentCollisionChecker(
      const VoxelizedEnvironmentCollisionChecker&);

 private:
  std::unique_ptr<drake::planning::CollisionChecker> DoClone() const override;

  std::optional<drake::geometry::GeometryId>
  AddEnvironmentCollisionShapeToBody(
      const std::string& group_name,
      const drake::multibody::Body<double>& bodyA,
      const drake::geometry::Shape& shape,
      const drake::math::RigidTransform<double>& X_AG) override;

  void RemoveAllAddedEnvironment(
      const std::vector<drake::planning::CollisionChecker::AddedShape>& shapes)
      override;

  /// Signed Distance Field models of the environment around the robot.
  std::map<std::string, EnvironmentSDFConstSharedPtr> environment_sdfs_;

  /// What body does each Signed Distance Field belong to?
  std::map<std::string, drake::multibody::BodyIndex> environment_sdf_bodies_;
};
}  // namespace planning
}  // namespace drake
