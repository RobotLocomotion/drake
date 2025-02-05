#pragma once

#include <Eigen/Geometry>

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace planning {
/// Wrapper type for position, velocity, and acceleration limits.
class JointLimits {
 public:
  /// Construct a JointLimits using the position, velocity, and acceleration
  /// limits in the provided `plant`. If set, throws if the position, velocity,
  /// or acceleration limits contain non-finite values. Position limits should
  /// generally be finite.
  JointLimits(const multibody::MultibodyPlant<double>& plant,
              bool require_finite_positions = false,
              bool require_finite_velocities = false,
              bool require_finite_accelerations = false);

  /// Construct a JointLimits using the position, velocity, and acceleration
  /// limits provided. If set, throws if the position, velocity, or acceleration
  /// limits contain non-finite values. Position limits should generally be
  /// finite.
  JointLimits(const Eigen::VectorXd& position_lower,
              const Eigen::VectorXd& position_upper,
              const Eigen::VectorXd& velocity_lower,
              const Eigen::VectorXd& velocity_upper,
              const Eigen::VectorXd& acceleration_lower,
              const Eigen::VectorXd& acceleration_upper,
              bool require_finite_positions = false,
              bool require_finite_velocities = false,
              bool require_finite_accelerations = false);

  JointLimits() {}

  int32_t num_positions() const {
    return static_cast<int32_t>(position_lower().size());
  }

  int32_t num_velocities() const {
    return static_cast<int32_t>(velocity_lower().size());
  }

  int32_t num_accelerations() const {
    return static_cast<int32_t>(acceleration_lower().size());
  }

  const Eigen::VectorXd& position_lower() const { return position_lower_; }

  const Eigen::VectorXd& position_upper() const { return position_upper_; }

  const Eigen::VectorXd& velocity_lower() const { return velocity_lower_; }

  const Eigen::VectorXd& velocity_upper() const { return velocity_upper_; }

  const Eigen::VectorXd& acceleration_lower() const {
    return acceleration_lower_;
  }

  const Eigen::VectorXd& acceleration_upper() const {
    return acceleration_upper_;
  }

  /// Checks if `position` is within position limits +/- `tolerance` beyond
  /// limits.
  bool CheckInPositionLimits(const Eigen::VectorXd& position,
                             double tolerance = 0.0) const;

  /// Checks if `velocity` is within velocity limits +/- `tolerance` beyond
  /// limits.
  bool CheckInVelocityLimits(const Eigen::VectorXd& velocity,
                             double tolerance = 0.0) const;

  /// Checks if `acceleration` is within acceleration limits +/- `tolerance`
  /// beyond limits.
  bool CheckInAccelerationLimits(const Eigen::VectorXd& acceleration,
                                 double tolerance = 0.0) const;

 private:
  Eigen::VectorXd position_lower_;
  Eigen::VectorXd position_upper_;

  Eigen::VectorXd velocity_lower_;
  Eigen::VectorXd velocity_upper_;

  Eigen::VectorXd acceleration_lower_;
  Eigen::VectorXd acceleration_upper_;
};

}  // namespace planning
}  // namespace drake
