#include "drake/multibody/plant/distance_constraint_params.h"

#include "drake/common/drake_throw.h"

namespace drake {
namespace multibody {

template <typename T>
DistanceConstraintParams<T>::DistanceConstraintParams(
    BodyIndex bodyA, const Vector3<T>& p_AP, BodyIndex bodyB,
    const Vector3<T>& p_BQ, const T& distance, const T& stiffness,
    const T& damping)
    : bodyA_(bodyA),
      p_AP_(p_AP),
      bodyB_(bodyB),
      p_BQ_(p_BQ),
      distance_(distance),
      stiffness_(stiffness),
      damping_(damping) {
  DRAKE_THROW_UNLESS(bodyA.is_valid());
  DRAKE_THROW_UNLESS(bodyB.is_valid());
  DRAKE_THROW_UNLESS(bodyA != bodyB);
  DRAKE_THROW_UNLESS(distance > 0.0);
  DRAKE_THROW_UNLESS(stiffness > 0.0);
  DRAKE_THROW_UNLESS(damping >= 0.0);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::DistanceConstraintParams);
