#include "drake/multibody/plant/distance_constraint_params.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {

DistanceConstraintParams::DistanceConstraintParams(
    BodyIndex bodyA, const Vector3<double>& p_AP, BodyIndex bodyB,
    const Vector3<double>& p_BQ, double distance, double stiffness,
    double damping)
    : bodyA_(bodyA),
      p_AP_(p_AP),
      bodyB_(bodyB),
      p_BQ_(p_BQ),
      distance_(distance),
      stiffness_(stiffness),
      damping_(damping) {
  if (!bodyA.is_valid()) {
    throw std::logic_error("Invalid index for body A.");
  }
  if (!bodyB.is_valid()) {
    throw std::logic_error("Invalid index for body B.");
  }
  if (bodyA == bodyB) {
    throw std::logic_error("Body indexes are equal.");
  }
  if (!(distance > 0.0)) {
    throw std::logic_error("Distance must be strictly positive.");
  }
  if (!(stiffness > 0.0)) {
    throw std::logic_error("Stiffness must be strictly positive.");
  }
  if (!(damping >= 0.0)) {
    throw std::logic_error("Damping must be positive or zero.");
  }
}

}  // namespace multibody
}  // namespace drake
