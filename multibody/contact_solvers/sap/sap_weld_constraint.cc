#include "drake/multibody/contact_solvers/sap/sap_weld_constraint.h"

#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"

using drake::math::RigidTransform;
using drake::math::RotationMatrix;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapWeldConstraint<T>::Kinematics::Kinematics(int objectA,
                                             RigidTransform<T> X_WP,
                                             Vector3<T> p_AP_W, int objectB,
                                             RigidTransform<T> X_WQ,
                                             Vector3<T> p_BQ_W,
                                             SapConstraintJacobian<T> J_AmBm_W)
    : objectA_(objectA),
      X_WP_(std::move(X_WP)),
      p_AP_W_(std::move(p_AP_W)),
      objectB_(objectB),
      X_WQ_(std::move(X_WQ)),
      p_BQ_W_(std::move(p_BQ_W)),
      J_(std::move(J_AmBm_W)) {
  // Thus far only dense Jacobian blocks are supported, i.e. rigid body
  // applications in mind.
  DRAKE_THROW_UNLESS(J_.blocks_are_dense());

  p_PoQo_W_ = X_WQ_.translation() - X_WP_.translation();
  const AngleAxis<T> R_PQ =
      X_WP_.rotation().InvertAndCompose(X_WQ_.rotation()).ToAngleAxis();

  a_PQ_W_ = X_WP.rotation() * (R_PQ.angle() * R_PQ.axis());
}

template <typename T>
SapWeldConstraint<T>::SapWeldConstraint(Kinematics kinematics)
    : SapHolonomicConstraint<T>(
          MakeSapHolonomicConstraintKinematics(kinematics),
          MakeSapHolonomicConstraintParameters(),
          {kinematics.objectA(), kinematics.objectB()}),
      kinematics_(std::move(kinematics)) {}

template <typename T>
typename SapHolonomicConstraint<T>::Parameters
SapWeldConstraint<T>::MakeSapHolonomicConstraintParameters() {
  // "Near-rigid" regime parameter, see [Castro et al., 2022].
  // TODO(amcastro-tri): consider exposing this parameter.
  constexpr double kBeta = 0.1;

  // Weld constraints do not have impulse limits, they are bi-lateral
  // constraints. Each weld constraint introduces 6 constraint
  // equations.
  //
  // N.B. We model weld constraints as "near-rigid" by setting stiffness to
  // infinity. relaxation_time is set to 0 and will be overwritten by
  // SapHolonomicConstraint when it detects the "near-rigid" case.
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const Vector6<T> infinity = kInf * Vector6<T>::Ones();
  return typename SapHolonomicConstraint<T>::Parameters{
      -infinity, infinity, infinity, Vector6<T>::Zero(), kBeta};
}

template <typename T>
typename SapHolonomicConstraint<T>::Kinematics
SapWeldConstraint<T>::MakeSapHolonomicConstraintKinematics(
    const Kinematics& kinematics) {
  // See class documentation for SapWeldConstraint for definitions of all
  // quantites referenced below.

  // Constraint function g.
  Vector6<T> g =
      (Vector6<T>() << kinematics.a_PQ_W(), kinematics.p_PoQo_W()).finished();
  Vector6<T> b = Vector6<T>::Zero();  // Bias term.

  // To write this holonomic constraint implicitly, we need to formulate
  // g(t + dt). Typically we just make use the first-order truncation of the
  // Taylor series for g(t):
  //   g(t + dt) = g(0) + dt⋅ʷdg/dt + O(dt²)
  // using vc = ʷdg/dt as the constraint velocity and accumulating a small error
  // of O(dt²). We stray from this approach for several reasons, outlined below.
  //
  // We can decompose constraint velocity into:
  //   ʷdg/dt = (a_PQ_W, v_W_PoQo).
  // First let's consider the translational component, v_W_PoQo. Choosing
  // this quantity directly would lead to an impulse applied at Po and an equal
  // and opposite impulse at Qo. This unfortunately does not conserve angular
  // momentum and introduces a small error of order O(‖γ‖⋅‖p_PQ‖), where ‖γ‖ is
  // the magnitude of the constraint impulse γ. We instead choose a velocity
  // that leads to an impulse that conserves angular momentum.
  //
  // With M the midpoint of Po and Qo, we define Am to be a material point on A
  // coincident with M and likewise Bm a material point on B, coincident with M.
  // We define the translational component of our constraint velocity as:
  //   vcₜ ≜ v_W_AmBm
  // We can write v_W_AmBm in terms of v_W_PoQo:
  //   v_W_AmBm = v_WBm - v_WAm
  //            = (v_WQo + w_WB×p_QoBm_W) - (v_WPo + w_WA×p_PoAm_W)
  //            = v_W_PoQo + (w_WB×(p_QoPo_W / 2) - w_WA×(p_PoQo_W / 2))
  //            = v_W_PoQo + p_PoQo_W×(w_WA + w_WB)/2
  // Then rearranging the translational component of our approximation:
  //   gₜ(t + dt) = gₜ(0) + dt⋅v_W_PoQo + O(dt²)
  //              = gₜ(0) + dt⋅(v_W_AmBm - p_PoQo_W×(w_WA + w_WB)/2) + O(dt²)
  //              = gₜ(0) + dt⋅v_W_AmBm + O(dt⋅‖p_PQ‖⋅‖w_WA + w_WB‖)
  //
  // Thus choosing v_W_AmBm as the translational component of vcₜ approximates
  // gₜ(t + dt) with an error order O(dt⋅‖p_PQ‖⋅‖w_WA + w_WB‖), which we can
  // expect to be small when the constraint is close to satisfied. We show
  // below (in DoAccumulateSpatialImpulses()) that this choice has the
  // additional benefit of conserving angular momentum.
  //
  // For the angular component:
  //   ᴺd(a_PQ)/dt = ᴺd(θk)/dt = ᴺdθ/dt⋅k + θ⋅ᴺdk/dt
  // From [Mitiguy 2022, §9.2.2]:
  //   θ⋅ᴮdk/dt = θ/2⋅[(cos(θ/2)/sin(θ/2))⋅(w_AB − ᴮdθ/dt⋅k) + k×w_AB)]
  // If we make a small angle assumption θ ≈ 0 to get:
  //   θ⋅ᴮdk/dt ≈ w_AB − ᴮdθ/dt⋅k + θ/2⋅k×w_AB
  // Thus:
  //   ᴮd(a_PQ)/dt ≈ ᴮdθ/dt⋅k + w_AB − ᴮdθ/dt⋅k + θ/2⋅k×w_AB
  //               = w_AB + θ/2⋅k×w_AB
  // Again making the small angle assumption and ignoring the θ dependent term:
  //   ᴮd(a_PQ)/dt ≈ w_AB
  //
  // N.B. We had a choice of which frame to measure dk/dt, but when we make the
  // small angle assumption the frame dependent terms drop out. Our
  // approximation is independent of any frame between P and Q, thus we can say
  // that measured and expressed in the world frame:
  //   ʷd(a_PQ)/dt ≈ w_AB
  //
  // Then by our approximations:
  //   vc = (ʷd(a_PQ)/dt, ʷd(p_PoQo)/dt) ≈ (w_AB_W, v_W_AmBm) = V_W_AmBm
  //
  // Thus the constraint Jacobian is the relative spatial velocity jacobian
  // between P and Q in the world frame:
  //  ġ = V_W_AmBm = J_W_AmBm ⋅ v
  //
  // [Mitiguy 2022] Mitiguy, P., 2022. Advanced Dynamics & Motion Simulation.

  return typename SapHolonomicConstraint<T>::Kinematics(
      std::move(g), std::move(kinematics.jacobian()), std::move(b));
}

template <typename T>
void SapWeldConstraint<T>::DoAccumulateSpatialImpulses(
    int i, const Eigen::Ref<const VectorX<T>>& gamma,
    SpatialForce<T>* F) const {
  // To interpret γ = (γᵣ, γₜ) as a spatial impulse and determine the point of
  // application for this formulation, let's consider the case where both A and
  // B are free bodies for simplicity. In this case the generalized velocities v
  // are just the spatial velocities of each body in the world frame stacked:
  //   v = [w_WA, v_WA, w_WB, v_WB]
  // We know that J⋅v = V_W_AmBm = V_WBm - V_WAm. Thus J is just the operator
  // that takes the difference of the body's spatial velocities measured and
  // expressed in world, and shifts to Am and Bm:
  //   J = [      -[I]      0          [I]    0]
  //       [[p_AoAm_W]ₓ  -[I]  -[p_BoBm_W]ₓ [I]]
  // We also know from the optimality condition of the SAP formulation
  // (essentially the balance of momentum condition):
  //   A⋅(v - v*) - Jᵀ⋅γ = 0
  // Therefore the generalized impulse Jᵀ⋅γ corresponds to a spatial impulse on
  // body A and a spatial impulse on body B stacked:
  //   Jᵀ⋅γ = [Γ_Ao_W, Γ_Bo_W]
  // Where:
  //   Γ_Ao_W = ((-γᵣ) - [p_AmAo]ₓ⋅(-γₜ), (-γₜ)) and
  //   Γ_Bo_W = (( γᵣ) - [p_BmBo]ₓ⋅( γᵣ), ( γₜ))
  // Therefore Jᵀ can be understood as the operator that takes a spatial impulse
  // -γ applied on body A at Am and shifts to Ao. As well it takes the equal and
  // opposite spatial impulse γ applied on body B at Bm and shifts to Bo. Thus,
  // this constraint can be interpreted as applying an impulse γ at point Bm on
  // B and an impulse -γ at point Am on A. As a consequence the constraint
  // impulse satisfies Newton's 3rd law and conserves angular momentum.
  if (i == 0) {
    // Object A.
    // Spatial impulse on A, applied at point Am, expressed in the world frame.
    const SpatialForce<T> gamma_AAm_W(-gamma.template head<3>(),
                                      -gamma.template tail<3>());
    const Vector3<T> p_AoAm_W =
        kinematics().p_AP_W() + 0.5 * kinematics().p_PoQo_W();
    *F += gamma_AAm_W.Shift(-p_AoAm_W);
  } else {
    // Object B.
    // Spatial impulse on B, applied at point Bm, expressed in the world frame.
    const SpatialForce<T> gamma_BBm_W(gamma.template head<3>(),
                                      gamma.template tail<3>());
    const Vector3<T> p_BoBm_W =
        kinematics().p_BQ_W() - 0.5 * kinematics().p_PoQo_W();
    *F += gamma_BBm_W.Shift(-p_BoBm_W);
    return;
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapWeldConstraint)
