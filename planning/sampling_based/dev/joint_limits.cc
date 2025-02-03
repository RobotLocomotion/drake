#include "planning/joint_limits.h"

#include <stdexcept>

#include <Eigen/Geometry>
#include <fmt/format.h>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace anzu {
namespace planning {
namespace {
void CheckForFiniteValues(
    const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
    std::string_view name, bool require_finite) {
  for (int32_t index = 0; index < lower.size(); ++index) {
    if (!std::isfinite(lower(index)) ||
        !std::isfinite(upper(index))) {
      if (require_finite) {
        throw std::runtime_error(fmt::format(
            "{}:{} limits [{}, {}] are not finite",
            name, index, lower(index), upper(index)));
      } else {
        drake::log()->warn(
            "{}:{} limits [{}, {}] are not finite, this will cause "
            "problems with trajectory parametrization",
            name, index, lower(index), upper(index));
      }
    }
  }
}
}  // namespace

JointLimits::JointLimits(
    const drake::multibody::MultibodyPlant<double>& plant,
    const bool require_finite_positions,
    const bool require_finite_velocities,
    const bool require_finite_accelerations)
    : JointLimits(
        plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits(),
        plant.GetVelocityLowerLimits(), plant.GetVelocityUpperLimits(),
        plant.GetAccelerationLowerLimits(), plant.GetAccelerationUpperLimits(),
        require_finite_positions, require_finite_velocities,
        require_finite_accelerations) {}

JointLimits::JointLimits(
    const Eigen::VectorXd& position_lower,
    const Eigen::VectorXd& position_upper,
    const Eigen::VectorXd& velocity_lower,
    const Eigen::VectorXd& velocity_upper,
    const Eigen::VectorXd& acceleration_lower,
    const Eigen::VectorXd& acceleration_upper,
    const bool require_finite_positions,
    const bool require_finite_velocities,
    const bool require_finite_accelerations)
    : position_lower_(position_lower),
      position_upper_(position_upper),
      velocity_lower_(velocity_lower),
      velocity_upper_(velocity_upper),
      acceleration_lower_(acceleration_lower),
      acceleration_upper_(acceleration_upper) {
  DRAKE_THROW_UNLESS(position_lower_.size() == position_upper_.size());
  DRAKE_THROW_UNLESS(velocity_lower_.size() == velocity_upper_.size());
  DRAKE_THROW_UNLESS(acceleration_lower_.size() == acceleration_upper_.size());

  // Position size and velocity size can differ (e.g. in mobile robots), but
  // velocity size and acceleration size must match.
  DRAKE_THROW_UNLESS(velocity_lower_.size() == acceleration_lower_.size());

  DRAKE_THROW_UNLESS(
      (position_lower_.array() <= position_upper_.array()).all());
  DRAKE_THROW_UNLESS(
      (velocity_lower_.array() <= velocity_upper_.array()).all());
  DRAKE_THROW_UNLESS(
      (acceleration_lower_.array() <= acceleration_upper_.array()).all());

  CheckForFiniteValues(
      this->position_lower(), this->position_upper(), "Position",
      require_finite_positions);
  CheckForFiniteValues(
      this->velocity_lower(), this->velocity_upper(), "Velocity",
      require_finite_velocities);
  CheckForFiniteValues(
      this->acceleration_lower(), this->acceleration_upper(), "Acceleration",
      require_finite_accelerations);
}

bool JointLimits::CheckInPositionLimits(
    const Eigen::VectorXd& position, const double tolerance) const {
  DRAKE_THROW_UNLESS(position.size() == position_lower().size());
  DRAKE_THROW_UNLESS(tolerance >= 0.0);

  return ((position.array() >= (position_lower().array() - tolerance)) &&
          (position.array() <= (position_upper().array() + tolerance))).all();
}

bool JointLimits::CheckInVelocityLimits(
    const Eigen::VectorXd& velocity, const double tolerance) const {
  DRAKE_THROW_UNLESS(velocity.size() == velocity_lower().size());
  DRAKE_THROW_UNLESS(tolerance >= 0.0);

  return ((velocity.array() >= (velocity_lower().array() - tolerance)) &&
          (velocity.array() <= (velocity_upper().array() + tolerance))).all();
}

bool JointLimits::CheckInAccelerationLimits(
    const Eigen::VectorXd& acceleration, const double tolerance) const {
  DRAKE_THROW_UNLESS(acceleration.size() == acceleration_lower().size());
  DRAKE_THROW_UNLESS(tolerance >= 0.0);

  return ((acceleration.array() >=
              (acceleration_lower().array() - tolerance)) &&
          (acceleration.array() <=
              (acceleration_upper().array() + tolerance))).all();
}

}  // namespace planning
}  // namespace anzu
