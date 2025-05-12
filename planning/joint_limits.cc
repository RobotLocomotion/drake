#include "planning/joint_limits.h"

#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace planning {

using Eigen::VectorXd;
using multibody::MultibodyPlant;

namespace {

void ValidateLimits(const VectorXd& lower, const VectorXd& upper,
                    std::string_view name, bool require_finite) {
  if (lower.size() != upper.size()) {
    throw std::runtime_error(
        fmt::format("{}: limits [size {}, size {}] must be the same size", name,
                    std::ssize(lower), std::ssize(upper)));
  }
  if (!(lower.array() <= upper.array()).all()) {
    throw std::runtime_error(
        fmt::format("{}: limits: no lower limit coefficient can "
                    "exceed its matching upper limit.\nlower:\n{}\nupper:\n{}",
                    name, fmt_eigen(lower), fmt_eigen(upper)));
  }

  for (int index = 0; index < std::ssize(lower); ++index) {
    if (!std::isfinite(lower(index)) || !std::isfinite(upper(index))) {
      if (require_finite) {
        throw std::runtime_error(
            fmt::format("{}:{} limits [{}, {}] are not finite", name, index,
                        lower(index), upper(index)));
      } else {
        log()->debug(
            "{}:{} limits [{}, {}] are not finite, this may cause "
            "problems with trajectory parametrization",
            name, index, lower(index), upper(index));
      }
    }
  }
}

bool CheckInLimits(const VectorXd& value, const VectorXd& lower,
                   const VectorXd& upper, const double tolerance) {
  DRAKE_THROW_UNLESS(tolerance >= 0.0);
  DRAKE_THROW_UNLESS(!(value.array().isNaN().any()));
  return ((value.array() >= (lower.array() - tolerance)) &&
          (value.array() <= (upper.array() + tolerance)))
      .all();
}

}  // namespace

JointLimits::JointLimits(const MultibodyPlant<double>& plant,
                         const bool require_finite_positions,
                         const bool require_finite_velocities,
                         const bool require_finite_accelerations)
    : JointLimits(
          plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits(),
          plant.GetVelocityLowerLimits(), plant.GetVelocityUpperLimits(),
          plant.GetAccelerationLowerLimits(),
          plant.GetAccelerationUpperLimits(), require_finite_positions,
          require_finite_velocities, require_finite_accelerations) {}

JointLimits::JointLimits(
    const VectorXd& position_lower, const VectorXd& position_upper,
    const VectorXd& velocity_lower, const VectorXd& velocity_upper,
    const VectorXd& acceleration_lower, const VectorXd& acceleration_upper,
    const bool require_finite_positions, const bool require_finite_velocities,
    const bool require_finite_accelerations)
    : position_{position_lower, position_upper},
      velocity_{velocity_lower, velocity_upper},
      acceleration_{acceleration_lower, acceleration_upper} {
  ValidateLimits(position_.lower, position_.upper, "Position",
                 require_finite_positions);
  ValidateLimits(velocity_.lower, velocity_.upper, "Velocity",
                 require_finite_velocities);
  ValidateLimits(acceleration_.lower, acceleration_.upper, "Acceleration",
                 require_finite_accelerations);

  // Position size and velocity size can differ (e.g. in mobile robots), but
  // velocity size and acceleration size must match.
  DRAKE_THROW_UNLESS(velocity_.lower.size() == acceleration_.lower.size());
}

JointLimits::~JointLimits() = default;

bool JointLimits::CheckInPositionLimits(const VectorXd& position,
                                        const double tolerance) const {
  DRAKE_THROW_UNLESS(position.size() == position_lower().size());
  return CheckInLimits(position, position_.lower, position_.upper, tolerance);
}

bool JointLimits::CheckInVelocityLimits(const VectorXd& velocity,
                                        const double tolerance) const {
  DRAKE_THROW_UNLESS(velocity.size() == velocity_lower().size());
  return CheckInLimits(velocity, velocity_.lower, velocity_.upper, tolerance);
}

bool JointLimits::CheckInAccelerationLimits(const VectorXd& acceleration,
                                            const double tolerance) const {
  DRAKE_THROW_UNLESS(acceleration.size() == acceleration_lower().size());
  return CheckInLimits(acceleration, acceleration_.lower, acceleration_.upper,
                       tolerance);
}

}  // namespace planning
}  // namespace drake
