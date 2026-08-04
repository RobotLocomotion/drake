#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/dof_mask.h"

namespace drake {
namespace planning {
/** Wrapper type for position, velocity, and acceleration limits.

Note that enforcement of finite limits by this class is optional; see the
`require_finite_*` constructor arguments.

NaNs are rejected for all limits and tolerances. */
class JointLimits final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(JointLimits);

  /** Constructs a %JointLimits using the position, velocity, and acceleration
  limits in the provided `plant`.
  @throws std::exception if plant is not finalized.
  @throws std::exception if the position, velocity, or acceleration limits
  contain non-finite values, and the corresponding constructor argument is
  true.
  @throws std::exception if any limit value is NaN.
  @pydrake_mkdoc_identifier{plant} */
  JointLimits(const multibody::MultibodyPlant<double>& plant,
              bool require_finite_positions = false,
              bool require_finite_velocities = false,
              bool require_finite_accelerations = false);

  /** Constructs a %JointLimits using the position, velocity, and acceleration
  limits in the provided `plant`, selecting only the dofs indicated by
  `active_dof.`
  @throws std::exception if plant is not finalized.
  @throws std::exception if active_dof.size() != num_positions().
  @throws std::exception if active_dof.size() != num_velocities().
  @throws std::exception if active_dof.size() != num_accelerations().
  @throws std::exception if the position, velocity, or acceleration limits
  contain non-finite values, and the corresponding constructor argument is
  true.
  @throws std::exception if any limit value is NaN.
  @pydrake_mkdoc_identifier{plant_select} */
  JointLimits(const multibody::MultibodyPlant<double>& plant,
              const DofMask& active_dof, bool require_finite_positions = false,
              bool require_finite_velocities = false,
              bool require_finite_accelerations = false);

  /** Constructs a %JointLimits using the position, velocity, and acceleration
  limits in the provided `other`, selecting only the coefficients indicated by
  `active_dof.`
  @throws std::exception if active_dof.size() != other.num_positions().
  @throws std::exception if active_dof.size() != other.num_velocities().
  @throws std::exception if active_dof.size() != other.num_accelerations().
  @throws std::exception if the position, velocity, or acceleration limits
  contain non-finite values, and the corresponding constructor argument is
  true.
  @throws std::exception if any limit value is NaN.
  @pydrake_mkdoc_identifier{copy_select} */
  JointLimits(const JointLimits& other, const DofMask& active_dof,
              bool require_finite_positions = false,
              bool require_finite_velocities = false,
              bool require_finite_accelerations = false);

  /** Constructs a %JointLimits using the provided arguments.
  @throws std::exception if the position, velocity, or acceleration limits
  contain non-finite values, and the corresponding constructor argument is
  true.
  @throws std::exception if any lower/upper limit pairs differ in size.
  @throws std::exception if any upper limit coefficients are less than the
  corresponding lower limit coefficient.
  @throws std::exception if velocity and acceleration limits differ in size.
  @throws std::exception if any limit value is NaN.
  @pydrake_mkdoc_identifier{vectors} */
  JointLimits(const Eigen::VectorXd& position_lower,
              const Eigen::VectorXd& position_upper,
              const Eigen::VectorXd& velocity_lower,
              const Eigen::VectorXd& velocity_upper,
              const Eigen::VectorXd& acceleration_lower,
              const Eigen::VectorXd& acceleration_upper,
              bool require_finite_positions = false,
              bool require_finite_velocities = false,
              bool require_finite_accelerations = false);

  /** Constructs a %JointLimits with 0-length vectors for all limits. */
  JointLimits() = default;

  ~JointLimits();

  int num_positions() const { return position_lower().size(); }

  int num_velocities() const { return velocity_lower().size(); }

  int num_accelerations() const { return acceleration_lower().size(); }

  const Eigen::VectorXd& position_lower() const { return position_.lower; }

  const Eigen::VectorXd& position_upper() const { return position_.upper; }

  const Eigen::VectorXd& velocity_lower() const { return velocity_.lower; }

  const Eigen::VectorXd& velocity_upper() const { return velocity_.upper; }

  const Eigen::VectorXd& acceleration_lower() const {
    return acceleration_.lower;
  }

  const Eigen::VectorXd& acceleration_upper() const {
    return acceleration_.upper;
  }

  /** Checks if `position` is within position limits, relaxed outward by
  `tolerance` in both directions.
  @throws std::exception if `position.size() != num_positions()`.
  @throws std::exception if `tolerance` is negative.
  @throws std::exception if `position` or `tolerance` contain NaN values. */
  bool CheckInPositionLimits(const Eigen::VectorXd& position,
                             double tolerance = 0.0) const;

  /** Checks if `velocity` is within velocity limits, relaxed outward by
  `tolerance` in both directions.
  @throws std::exception if `velocity.size() != num_velocities()`.
  @throws std::exception if `tolerance` is negative.
  @throws std::exception if `velocity` or `tolerance` contain NaN values. */
  bool CheckInVelocityLimits(const Eigen::VectorXd& velocity,
                             double tolerance = 0.0) const;

  /** Checks if `acceleration` is within acceleration limits, relaxed outward
  by `tolerance` in both directions.
  @throws std::exception if `acceleration.size() != num_accelerations()`.
  @throws std::exception if `tolerance` is negative.
  @throws std::exception if `acceleration` or `tolerance` contain NaN values. */
  bool CheckInAccelerationLimits(const Eigen::VectorXd& acceleration,
                                 double tolerance = 0.0) const;

 private:
  struct Limits {
    Eigen::VectorXd lower;
    Eigen::VectorXd upper;
  };
  Limits position_;
  Limits velocity_;
  Limits acceleration_;
};

}  // namespace planning
}  // namespace drake
