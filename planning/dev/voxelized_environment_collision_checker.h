#pragma once

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "drake/planning/collision_checker.h"
#include "drake/planning/collision_checker_params.h"
#include "drake/planning/dev/sphere_robot_model_collision_checker.h"
#include "drake/planning/dev/voxel_occupancy_map.h"
#include "drake/planning/dev/voxel_signed_distance_field.h"
#include "drake/planning/dev/voxel_tagged_object_occupancy_map.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace planning {

/// Collision checker using a voxelized environment model.
class VoxelizedEnvironmentCollisionChecker final
    : public SphereRobotModelCollisionChecker {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  // N.B. The copy constructor is protected for use in implementing Clone().
  void operator=(const VoxelizedEnvironmentCollisionChecker&) = delete;
  /** @} */

  /// Creates a new checker with the given params.
  explicit VoxelizedEnvironmentCollisionChecker(CollisionCheckerParams params);

  /// Update the voxelized environment.
  /// @param environment_name Name of the environment model to update. If the
  /// name is already in use, the new model replaces the old. To remove a model,
  /// provide a default-constructed VoxelOccupancyMap that is not initialized.
  /// @param environment Voxelized environment model.
  /// @param override_environment_body_index Optionally provide a body index to
  /// override the environment frame name -> body lookup. Use this if the frame
  /// name is not unique, or if the frame name does not match an existing MbP
  /// body.
  void UpdateEnvironment(const std::string& environment_name,
                         const VoxelOccupancyMap& environment,
                         const std::optional<multibody::BodyIndex>&
                             override_environment_body_index = {});

  /// Update the voxelized environment.
  /// @param environment_name Name of the environment model to update. If the
  /// name is already in use, the new model replaces the old. To remove a model,
  /// provide a default-constructed VoxelTaggedObjectOccupancyMap that is not
  /// initialized.
  /// @param environment Voxelized environment model.
  /// @param override_environment_body_index Optionally provide a body index to
  /// override the environment frame name -> body lookup. Use this if the frame
  /// name is not unique, or if the frame name does not match an existing MbP
  /// body.
  void UpdateEnvironment(const std::string& environment_name,
                         const VoxelTaggedObjectOccupancyMap& environment,
                         const std::optional<multibody::BodyIndex>&
                             override_environment_body_index = {});

  /// Update the voxelized environment.
  /// @param environment_name Name of the environment model to update. If the
  /// name is already in use, the new model replaces the old. To remove a model,
  /// provide a default-constructed VoxelSignedDistanceField.
  /// @param environment_sdf signed distance field of voxelized environment.
  /// @param override_environment_body_index Optionally provide a body index to
  /// override the environment frame name -> body lookup. Use this if the frame
  /// name is not unique, or if the frame name does not match an existing MbP
  /// body.
  void UpdateEnvironment(const std::string& environment_name,
                         const VoxelSignedDistanceField& environment_sdf,
                         const std::optional<multibody::BodyIndex>&
                             override_environment_body_index = {});

  /// Remove the voxelized model corresponding to `environment_name`.
  bool RemoveEnvironment(const std::string& environment_name);

  const std::map<std::string, VoxelSignedDistanceField>& EnvironmentSDFs()
      const {
    return environment_sdfs_;
  }

  const std::map<std::string, multibody::BodyIndex>& EnvironmentSDFBodies()
      const {
    return environment_sdf_bodies_;
  }

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
      const systems::Context<double>& plant_context,
      const geometry::QueryObject<double>& query_object,
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
      const systems::Context<double>& plant_context,
      const geometry::QueryObject<double>& query_object,
      const Eigen::Vector4d& p_WQ, double query_radius,
      const std::vector<Eigen::Isometry3d>& X_WB_set,
      const std::vector<Eigen::Isometry3d>& X_WB_inverse_set) const override;

 protected:
  /// To support Clone(), allow copying (but not move nor assign).
  explicit VoxelizedEnvironmentCollisionChecker(
      const VoxelizedEnvironmentCollisionChecker&);

 private:
  std::unique_ptr<CollisionChecker> DoClone() const override;

  std::optional<geometry::GeometryId> AddEnvironmentCollisionShapeToBody(
      const std::string& group_name, const multibody::Body<double>& bodyA,
      const geometry::Shape& shape,
      const math::RigidTransform<double>& X_AG) override;

  void RemoveAllAddedEnvironment(
      const std::vector<CollisionChecker::AddedShape>& shapes) override;

  std::optional<double> EstimateConservativePointToEnvironmentSignedDistance(
      const systems::Context<double>& context,
      const geometry::QueryObject<double>& query_object,
      const Eigen::Vector4d& p_WQ, double query_radius,
      const std::vector<Eigen::Isometry3d>& X_WB_set,
      const std::vector<Eigen::Isometry3d>& X_WB_inverse_set) const override;

  /// Signed Distance Field models of the environment around the robot.
  std::map<std::string, VoxelSignedDistanceField> environment_sdfs_;

  /// What body does each Signed Distance Field belong to?
  std::map<std::string, multibody::BodyIndex> environment_sdf_bodies_;
};

}  // namespace planning
}  // namespace drake
