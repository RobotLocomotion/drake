#include "drake/multibody/contact_solvers/sap/sap_ball_constraint.h"

#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapBallConstraint<T>::SapBallConstraint(Kinematics kinematics)
    : SapHolonomicConstraint<T>(
          MakeSapHolonomicConstraintKinematics(kinematics),
          MakeSapHolonomicConstraintParameters(),
          {kinematics.objectA(), kinematics.objectB()}),
      kinematics_(std::move(kinematics)) {}

template <typename T>
typename SapHolonomicConstraint<T>::Parameters
SapBallConstraint<T>::MakeSapHolonomicConstraintParameters() {
  // "Near-rigid" regime parameter, see [Castro et al., 2022].
  // TODO(amcastro-tri): consider exposing this parameter.
  constexpr double kBeta = 0.1;

  // Ball constraints do not have impulse limits, they are bi-lateral
  // constraints. Each ball constraint introduces 3 constraint
  // equations. We set stiffness to infinity to indicate that this
  // constraint is modeled as near-rigid. Relaxation time is set to 0.
  // The time step will be used as an effective relaxation time when
  // SapHolonomicConstriant detects the near-rigid condition in DoMakeData().
  const Vector3<T> kInf =
      std::numeric_limits<double>::infinity() * Vector3<T>::Ones();
  return typename SapHolonomicConstraint<T>::Parameters{
      -kInf, kInf, kInf, Vector3<T>::Zero(), kBeta};
}

template <typename T>
typename SapHolonomicConstraint<T>::Kinematics
SapBallConstraint<T>::MakeSapHolonomicConstraintKinematics(
    const Kinematics& kinematics) {
  Vector3<T> g(kinematics.p_WQ() - kinematics.p_WP());  // Constraint function.
  Vector3<T> b = Vector3<T>::Zero();                    // Bias term.

  return typename SapHolonomicConstraint<T>::Kinematics(
      std::move(g), kinematics.jacobian(), std::move(b));
}

template <typename T>
void SapBallConstraint<T>::DoAccumulateSpatialImpulses(
    int i, const Eigen::Ref<const VectorX<T>>& gamma,
    SpatialForce<T>* F) const {
  if (i == 0) {
    // Object A.
    // -gamma = gamma_AN_W
    const SpatialForce<T> F_AN_W(Vector3<T>::Zero(), -gamma);
    *F += F_AN_W.Shift(kinematics().p_NA_W());
  } else {
    // Object B.
    // gamma = gamma_BN_W
    const SpatialForce<T> F_BN_W(Vector3<T>::Zero(), gamma);
    *F += F_BN_W.Shift(kinematics().p_NB_W());
    return;
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapBallConstraint)
