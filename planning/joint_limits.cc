#include "drake/planning/joint_limits.h"

#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
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
                    lower.size(), upper.size()));
  }
  auto limits_as_matrix = [&]() {
    Eigen::MatrixXd combined(lower.size(), 2);
    combined.col(0) = lower;
    combined.col(1) = upper;
    return combined;
  };
  if (!(lower.array() <= upper.array()).all()) {
    throw std::runtime_error(
        fmt::format("{} limits invalid. Lower values must be <= upper "
                    "values.\n[lower|upper]:\n{}",
                    name, fmt_eigen(limits_as_matrix())));
  }

  if (!lower.array().isFinite().all() || !upper.array().isFinite().all()) {
    if (require_finite) {
      throw std::runtime_error(
          fmt::format("{} limits are not finite.\n[lower|upper]:\n{}", name,
                      fmt_eigen(limits_as_matrix())));
    } else {
      log()->debug(
          "{} limits are not finite, this may cause "
          "problems with trajectory parametrization.\n[lower|upper]:\n{}",
          name, fmt_eigen(limits_as_matrix()));
    }
  }
}

bool CheckInLimits(const VectorXd& value, const VectorXd& lower,
                   const VectorXd& upper, const double tolerance) {
  DRAKE_THROW_UNLESS(value.size() == lower.size());
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

JointLimits::JointLimits(const MultibodyPlant<double>& plant,
                         const DofMask& active_dof,
                         const bool require_finite_positions,
                         const bool require_finite_velocities,
                         const bool require_finite_accelerations)
    : JointLimits(active_dof.GetFromArray(plant.GetPositionLowerLimits()),
                  active_dof.GetFromArray(plant.GetPositionUpperLimits()),
                  active_dof.GetFromArray(plant.GetVelocityLowerLimits()),
                  active_dof.GetFromArray(plant.GetVelocityUpperLimits()),
                  active_dof.GetFromArray(plant.GetAccelerationLowerLimits()),
                  active_dof.GetFromArray(plant.GetAccelerationUpperLimits()),
                  require_finite_positions, require_finite_velocities,
                  require_finite_accelerations) {}

JointLimits::JointLimits(const JointLimits& other, const DofMask& active_dof,
                         const bool require_finite_positions,
                         const bool require_finite_velocities,
                         const bool require_finite_accelerations)
    : JointLimits(active_dof.GetFromArray(other.position_lower()),
                  active_dof.GetFromArray(other.position_upper()),
                  active_dof.GetFromArray(other.velocity_lower()),
                  active_dof.GetFromArray(other.velocity_upper()),
                  active_dof.GetFromArray(other.acceleration_lower()),
                  active_dof.GetFromArray(other.acceleration_upper()),
                  require_finite_positions, require_finite_velocities,
                  require_finite_accelerations) {}

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
  return CheckInLimits(position, position_.lower, position_.upper, tolerance);
}

bool JointLimits::CheckInVelocityLimits(const VectorXd& velocity,
                                        const double tolerance) const {
  return CheckInLimits(velocity, velocity_.lower, velocity_.upper, tolerance);
}

bool JointLimits::CheckInAccelerationLimits(const VectorXd& acceleration,
                                            const double tolerance) const {
  return CheckInLimits(acceleration, acceleration_.lower, acceleration_.upper,
                       tolerance);
}

}  // namespace planning
}  // namespace drake
