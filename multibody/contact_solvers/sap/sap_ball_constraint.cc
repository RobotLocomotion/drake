#include "drake/multibody/contact_solvers/sap/sap_ball_constraint.h"

#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/math/autodiff.h"

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
  // To interpret γ as a spatial impulse and determine the point of application
  // for this formulation, let's consider the case where both A and B are free
  // bodies for simplicity. In this case the generalized velocities v are just
  // the spatial velocities of each body in the world frame stacked:
  //   v = [w_WA, v_WA, w_WB, v_WB]
  // We know that J⋅v = v_W_PQ = v_WQ - v_WP. Thus J is just the operator that
  // takes the difference of the body's translational velocities shifted to P
  // and Q:
  //   J = [[p_AP]ₓ -[I] -[p_BQ]ₓ [I]]
  // We also know from the optimality condition of the SAP formulation
  // (essentially the balance of momentum condition):
  //   A⋅(v - v*) - Jᵀ⋅γ = 0
  // Therefore the generalized impulse Jᵀ⋅γ corresponds to a spatial impulse on
  // body A and a spatial impulse on body B stacked:
  //   Jᵀ⋅γ = [Γ_Ao_W, Γ_Bo_W]
  // Where:
  //   Γ_Ao_W = (-[p_PA]ₓ⋅-γ, -γ) and Γ_Bo_W = (-[p_QB]ₓ⋅γ, γ)
  // Therefore Jᵀ can be understood as the operator that shifts a spatial
  // impulse (0, -γ) applied at P to Ao and shifts the equal and opposite
  // spatial impulse (0, γ) applied at Q to Bo. Thus, this constraint can be
  // interpreted as applying an impulse γ at point Q on B and an impulse -γ
  // at point P on A. As a consequence the constraint satisfies Newton's 3rd
  // law, but does introduce a small moment of order O(‖γ‖⋅‖p_PQ‖) when P and Q
  // are not coincident.
  if (i == 0) {
    // Object A.
    // -gamma = gamma_Ap_W
    // Shift gamma_Ap_W = to Ao and add in.
    const SpatialForce<T> gamma_Ap_W(Vector3<T>::Zero(), -gamma);
    *F += gamma_Ap_W.Shift(-kinematics().p_AP_W());
  } else {
    // Object B.
    // gamma = gamma_Bq_W
    // Shift gamma_Bq_W to Bo and add in.
    const SpatialForce<T> gamma_Bq_W(Vector3<T>::Zero(), gamma);
    *F += gamma_Bq_W.Shift(-kinematics().p_BQ_W());
    return;
  }
}

template <typename T>
std::unique_ptr<SapConstraint<double>> SapBallConstraint<T>::DoToDouble()
    const {
  SapConstraintJacobian<double> J = this->jacobian().ToDouble();
  SapBallConstraint<double>::Kinematics k(
      kinematics_.objectA(), math::DiscardGradient(kinematics_.p_WP()),
      math::DiscardGradient(kinematics_.p_AP_W()), kinematics_.objectB(),
      math::DiscardGradient(kinematics_.p_WQ()),
      math::DiscardGradient(kinematics_.p_BQ_W()), std::move(J));
  return std::make_unique<SapBallConstraint<double>>(std::move(k));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapBallConstraint);
