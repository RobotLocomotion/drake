#include "drake/planning/collision_avoidance.h"

#include <algorithm>
#include <cmath>

namespace drake {
namespace planning {
namespace internal {

Eigen::VectorXd ComputeCollisionAvoidanceDisplacement(
    const CollisionChecker& checker, const Eigen::VectorXd& q,
    double max_penetration, double max_clearance,
    const std::optional<int> context_number, CollisionCheckerContext* context) {
  DRAKE_THROW_UNLESS(max_penetration <= 0.0);
  DRAKE_THROW_UNLESS(std::isfinite(max_penetration));
  DRAKE_THROW_UNLESS(max_clearance >= 0.0);
  DRAKE_THROW_UNLESS(std::isfinite(max_clearance));
  DRAKE_THROW_UNLESS(max_clearance > max_penetration);
  DRAKE_THROW_UNLESS(context_number == std::nullopt || context == nullptr);

  const double penetration_range = max_clearance - max_penetration;
  // Collect the distances and Jacobians.
  const RobotClearance robot_clearance =
      context == nullptr
          ? checker.CalcRobotClearance(q, max_clearance, context_number)
          : checker.CalcContextRobotClearance(context, q, max_clearance);

  const int num_measurements = robot_clearance.size();

  if (num_measurements == 0) {
    // No collisions within the safety range; so no displacement required.
    return Eigen::VectorXd::Zero(q.size());
  }

  // Compute weights based on distance.
  Eigen::VectorXd weights(num_measurements);
  for (int row = 0; row < num_measurements; ++row) {
    const double distance = robot_clearance.distances()(row);
    const double penetration = max_clearance - distance;
    weights(row) = std::clamp(penetration / penetration_range, 0.0, 1.0);
  }

  // Here we compute ∇f = Σᵢ(wᵢ⋅Jqᵣ_ϕᵢ)ᵀ. Per the RobotClearance API docs we
  // have ϕᵢ as the iᵗʰ distance measurement and Jqᵣ_ϕᵢ as the partial of ϕᵢ
  // with respect to qᵣ, the robot state; wᵢ is our weight for iᵗʰ measurement.
  //
  // Note that Jqᵣ_ϕ is dimensioned as (num_measurements, nq) and weights is
  // (num_measurements, 1), so we can use Jᵀ as a convenient way to calculate
  // the sum in a single pass.
  return robot_clearance.jacobians().transpose() * weights;
}
}  // namespace internal
}  // namespace planning
}  // namespace drake
