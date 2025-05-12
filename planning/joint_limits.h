#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace planning {
/** Wrapper type for position, velocity, and acceleration limits. Many planning
algorithms require that all position limits are finite. Unless the algorithm
explicitly documents support for infinite limits, assume that limits must be
finite. NaN values are rejected for all limits and tolerances.
*/
class JointLimits final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(JointLimits);

  /** Constructs a JointLimits using the position, velocity, and acceleration
  limits in the provided `plant`.
  @throws std::exception if the position, velocity, or acceleration limits
  contain non-finite values, and the corresponding constructor argument is
  true.
  */
  JointLimits(const multibody::MultibodyPlant<double>& plant,
              bool require_finite_positions = false,
              bool require_finite_velocities = false,
              bool require_finite_accelerations = false);

  /** Constructs a JointLimits using the provided arguments.
  @throws std::exception if the position, velocity, or acceleration limits
  contain non-finite values, and the corresponding constructor argument is
  true.
  @throws std::exception if any lower/upper limit pairs differ in size.
  @throws std::exception if any upper limit coefficients are less than the
  corresponding lower limit coefficient.
  @throws std::exception if velocity and acceleration limits differ in size.
  */
  JointLimits(const Eigen::VectorXd& position_lower,
              const Eigen::VectorXd& position_upper,
              const Eigen::VectorXd& velocity_lower,
              const Eigen::VectorXd& velocity_upper,
              const Eigen::VectorXd& acceleration_lower,
              const Eigen::VectorXd& acceleration_upper,
              bool require_finite_positions = false,
              bool require_finite_velocities = false,
              bool require_finite_accelerations = false);

  /** Constructs a JointLimits with 0-length vectors for all limits. */
  JointLimits() = default;

  ~JointLimits();

  int num_positions() const { return std::ssize(position_lower()); }

  int num_velocities() const { return std::ssize(velocity_lower()); }

  int num_accelerations() const { return std::ssize(acceleration_lower()); }

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

  /** Checks if `position` is within position limits +/- `tolerance` beyond
  limits.
  @throws std::exception if `position` differs in size from `num_positions()`.
  @throws std::exception if `tolerance` is negative. */
  bool CheckInPositionLimits(const Eigen::VectorXd& position,
                             double tolerance = 0.0) const;

  /** Checks if `velocity` is within velocity limits +/- `tolerance` beyond
  limits.
  @throws std::exception if `velocity` differs in size from `num_velocities()`.
  @throws std::exception if `tolerance` is negative. */
  bool CheckInVelocityLimits(const Eigen::VectorXd& velocity,
                             double tolerance = 0.0) const;

  /** Checks if `acceleration` is within acceleration limits +/- `tolerance`
  beyond limits.
  @throws std::exception if `acceleration` differs in size from
  `num_accelerations()`.
  @throws std::exception if `tolerance` is negative. */
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
