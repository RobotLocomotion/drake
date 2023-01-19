#include "drake/planning/collision_avoidance.h"

#include <algorithm>
#include <cmath>

namespace drake {
namespace planning {
namespace internal {

Eigen::VectorXd ComputeCollisionAvoidanceDisplacement(
    const CollisionChecker& checker, const Eigen::VectorXd& q,
    double max_penetration, double max_clearance,
    CollisionCheckerContext* context) {
  DRAKE_THROW_UNLESS(max_penetration <= 0.0);
  DRAKE_THROW_UNLESS(std::isfinite(max_penetration));
  DRAKE_THROW_UNLESS(max_clearance >= 0.0);
  DRAKE_THROW_UNLESS(std::isfinite(max_clearance));
  DRAKE_THROW_UNLESS(max_clearance > max_penetration);

  const double penetration_range = max_clearance - max_penetration;
  // Collect the distances and Jacobians.
  const RobotClearance robot_clearance =
      context == nullptr
          ? checker.CalcRobotClearance(q, max_clearance)
          : checker.CalcContextRobotClearance(context, q, max_clearance);

  const int num_rows = robot_clearance.size();

  if (num_rows == 0) {
    // No collisions within the safety range; so no displacement required.
    return Eigen::VectorXd::Zero(q.size());
  }

  // Compute weights based on distance.
  Eigen::VectorXd weights(num_rows);
  for (int row = 0; row < num_rows; ++row) {
    const double distance = robot_clearance.distances()(row);
    const double penetration = max_clearance - distance;
    weights(row) = std::clamp(penetration / penetration_range, 0.0, 1.0);
  }

  // ∂𝐀/∂q = Σᵢ(wᵢ⋅Jqᵣ_ϕᵢ) where wᵢ is the weight for iᵗʰ measurement.
  return robot_clearance.jacobians().transpose() * weights;
}
}  // namespace internal
}  // namespace planning
}  // namespace drake
