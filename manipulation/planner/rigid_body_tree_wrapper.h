#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/manipulation/planner/kinematic_tree.h"
#include "drake/math/transform.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace manipulation {
namespace planner {

/** Implementation of KinematicTree that uses a pointer to an instance of
 * RigidBodyTree to perform kinematics computations. */
class RigidBodyTreeWrapper final : public KinematicTree {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBodyTreeWrapper);

  RigidBodyTreeWrapper() = delete;

  /** Constructs an instance using `tree`. The lifetime of `tree` must exceed
   * the lifetime of the RigidBodyTreeWrapper. */
  explicit RigidBodyTreeWrapper(const RigidBodyTree<double>* tree);

  /** Constructs an instance using `tree`. The newly constructed
   * RigidBodyTreeWrapper takes over ownership of `tree`. */
  explicit RigidBodyTreeWrapper(
      std::unique_ptr<RigidBodyTree<double>> tree);

  ~RigidBodyTreeWrapper();

  int num_positions() const override;

  int num_velocities() const override;

  int PositionStartIndexForBody(const std::string& body_name) const override;

  const drake::VectorX<double>& joint_position_lower_limit() const override;

  const drake::VectorX<double>& joint_position_upper_limit() const override;

  const drake::VectorX<double>& joint_velocity_lower_limit() const override;

  const drake::VectorX<double>& joint_velocity_upper_limit() const override;

  drake::VectorX<double> GetZeroConfiguration() const override;

  drake::VectorX<double> GetRandomConfiguration(
      std::default_random_engine* generator) const override;

  drake::math::Transform<double> CalcRelativeTransform(
      const drake::VectorX<double>& q, const std::string& frame_A_name,
      const std::string& frame_B_name) const override;

  std::shared_ptr<drake::solvers::Constraint> MakeCollisionAvoidanceConstraint(
      double collision_avoidance_threshold) const override;

  std::shared_ptr<drake::solvers::Constraint> MakeRelativePoseConstraint(
      const std::string& frame_A, const std::string& frame_B,
      const drake::math::Transform<double>& X_AB,
      double orientation_tolerance = 0,
      double position_tolerance = 0) const override;

 protected:
  void DoSetJointPositionLimits(int position_index, double lower_limit,
                                double upper_limit) override;
  void DoSetJointVelocityLimits(int velocity_index, double lower_limit,
                                double upper_limit) override;

 private:
  std::unique_ptr<RigidBodyTree<double>> owned_tree_{};
  const RigidBodyTree<double>* tree_{};
  drake::VectorX<double> joint_velocity_lower_limit_{};
  drake::VectorX<double> joint_velocity_upper_limit_{};
  // Position limits are stored independently so that they can be modified
  // without changing `tree_`.
  drake::VectorX<double> joint_position_lower_limit_{};
  drake::VectorX<double> joint_position_upper_limit_{};
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
