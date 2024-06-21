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
  const AngleAxis<T> a_PQ =
      X_WP_.rotation().InvertAndCompose(X_WQ_.rotation()).ToAngleAxis();

  // N.B. a_PQ has the same components when expressed in frame P
  // or frame Q. i.e. X_WP * a_PQ == X_WQ * a_PQ.
  a_PQ_W_ = X_WP.rotation() * (a_PQ.angle() * a_PQ.axis());
}

template <typename T>
bool SapWeldConstraint<T>::Kinematics::operator==(
    const Kinematics& other) const {
  if (objectA() != other.objectA()) return false;
  if (!X_WP().IsExactlyEqualTo(other.X_WP())) return false;
  if (p_AP_W() != other.p_AP_W()) return false;
  if (objectB() != other.objectB()) return false;
  if (!X_WQ().IsExactlyEqualTo(other.X_WQ())) return false;
  if (p_BQ_W() != other.p_BQ_W()) return false;
  if (jacobian() != other.jacobian()) return false;
  if (p_PoQo_W() != other.p_PoQo_W()) return false;
  if (a_PQ_W() != other.a_PQ_W()) return false;
  return true;
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
  const Vector6<T> infinity = Vector6<T>::Constant(kInf);
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

  // The SAP formulation requires the constraint to define a convex cost ℓ(vc)
  // that penalizes the constraint error function. The "key" to model a weld
  // constraint is to write a frame invariant cost, so that the formulation
  // conserves angular momentum. This is similar to the application of Noether's
  // theorem to a continuous system.
  // Given the SAP formulation is for a discrete system, the cost we define is
  // an "incremental potential" that only applies to the given time step. In
  // this setting, once the constraint velocity vc and cost ℓ(vc) are defined,
  // impulses follow from:
  //  γ = -dℓ/dvc
  // and the mulitbody spatial forces modeled by this cost are inferred from the
  // optimality conditions (the momentum balance), see
  // DoAccumulateSpatialImpulses().
  //
  // With this framework in mind, we define the constraint velocity vc as the
  // relative velocity of instantaneously coincident points Am and Bm located at
  // M, the midmpoint between P and Q
  //   vc = v_AmBm
  // And SAP cost as:
  //   ℓ(vc) = ℓₜ(vc) + ℓᵣ(vc)
  // Where we split the cost into a translational contribution ℓₜ(vc) and a
  // rotational contribution ℓᵣ(vc)
  //   ℓₜ(vc) = 1/2||p_PQ(t) + δt⋅v_AmBm||ₜ²,  ||x||ₜ² = Rₜ||x||²
  //   ℓᵣ(vc) = 1/2||a_PQ(t) + δt⋅w_AB||ᵣ²  ,  ||x||ᵣ² = Rᵣ||x||²
  // were a_PQ is the axis-angle vector and, Rₜ and Rᵣ the regularization
  // parameters determined by the SAP formulation in the "near-rigid" regime,
  // see SapHolonomicConstraint.
  // We can show (see below), that this formulation is an approximation to
  //   ℓₜ(vc) = 1/2||p_PQ(t+δt)||ₜ²
  //   ℓᵣ(vc) = 1/2||a_PQ(t+δt)||ᵣ²
  // which penalizes a deviation of g(vc) = [a_PQ, p_PQ] from zero and enjoys
  // from the following properties:
  //   1. with vc frame invariant, the SAP cost is frame invariant,
  //   2. the resulting impulses conserve angular momentum and,
  //   3. the resulting impulses satisfy Newton's third law.
  //
  //                            Derivation
  // For translational component, from the SAP formulation:
  //   γₜ = -dℓ/dvcₜ = -Rₜ(p_PQ(t) + δt⋅v_AmBm)
  // We show below that:
  //  p_PQ(t+δt) = p_PQ(t) + δt⋅v_AmBm + O(dt⋅‖p_PQ‖⋅‖w_WA + w_WB‖)
  // and therefore this models a linear spring on p_PQ(t+δt) (heavilty
  // overdamped given the time scale introduced by SAP's choice of Rₜ cannot
  // be resolved by the discrete time step δt).
  //
  // We can write v_W_AmBm in terms of v_W_PQ:
  //   v_AmBm = v_WBm - v_WAm
  //          = (v_WQ + w_WB×p_QBm_W) - (v_WP + w_WA×p_PAm_W)
  //          = v_W_PQ + (w_WB×(p_QP_W / 2) - w_WA×(p_PQ_W / 2))
  //          = v_W_PQ + p_PQ_W×(w_WA + w_WB)/2
  // And therefore we arrive to the conclusion that:
  //   p_PQ(t) + δt⋅v_AmBm ≈ p_PQ(t+δt)
  //                       = p_PQ(t) + δt⋅ᵂd(p_PQ)/t + O(δt²)
  //                       = p_PQ(t) + O(dt⋅‖p_PQ‖⋅‖w_WA + w_WB‖)
  // and thus the cost ℓₜ(vc) penalizes a first order approximation to
  // p_PQ(t+δt).
  // We show conservation of angular momentum in DoAccumulateSpatialImpulses().
  //
  // For the angular component, from the SAP formulation:
  //   γᵣ = -dℓ/dvcᵣ = -Rᵣ(a_PQ(t) + δt⋅w_AB)
  // We show below that for small values of θ = ||a_PQ(t)||, we have that
  //  a_PQ(t) + δt⋅w_AB ≈ a_PQ(t+δt)
  // and therefore this cost models a linear torsionial spring that oposes
  // deviations of a_PQ from zero.
  //
  // To show this we start with:
  //   ᴮd(a_PQ)/dt = ᴮd(θk)/dt = ᴮdθ/dt⋅k + θ⋅ᴮdk/dt
  // where we made the split a_PQ = θk and, without loss of generality, we
  // chose frame B for the time derivatives.
  // From [Mitiguy 2022, §9.2.2]:
  //   θ⋅ᴮdk/dt = θ/2⋅[(cos(θ/2)/sin(θ/2))⋅(w_AB − ᴮdθ/dt⋅k) + k×w_AB)]
  // If we make a small angle assumption θ ≈ 0 to get:
  //   θ⋅ᴮdk/dt ≈ w_AB − ᴮdθ/dt⋅k + θ/2⋅k×w_AB
  // Thus:
  //   ᴮd(a_PQ)/dt ≈ ᴮdθ/dt⋅k + w_AB − ᴮdθ/dt⋅k + θ/2⋅k×w_AB
  //               = w_AB + θ/2⋅k×w_AB
  // Again making the small angle assumption, we can neglect the θ dependent
  // term
  //   ᴮd(a_PQ)/dt ≈ w_AB
  // Had we chosen frame A to measured derivatives in, the first order
  // approximation would be the same.
  //
  // With this result, we can now write
  //   a_PQ(t+δt) ≈ a_PQ(t) + δt⋅w_AB
  // which is the term penalized by the constraint.
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
  // impulse satisfies Newton's 3rd law and, since the application point is
  // coincident, it conserves angular momentum.
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

template <typename T>
std::unique_ptr<SapConstraint<double>> SapWeldConstraint<T>::DoToDouble()
    const {
  const typename SapWeldConstraint<T>::Kinematics& k = kinematics_;
  auto discard_gradient = [](const RigidTransform<T>& X) {
    return RigidTransform<double>(
        RotationMatrix<double>(math::DiscardGradient(X.rotation().matrix())),
        math::DiscardGradient(X.translation()));
  };
  SapWeldConstraint<double>::Kinematics k_to_double(
      k.objectA(), discard_gradient(k.X_WP()),
      math::DiscardGradient(k.p_AP_W()), k.objectB(),
      discard_gradient(k.X_WQ()), math::DiscardGradient(k.p_BQ_W()),
      k.jacobian().ToDouble());
  return std::make_unique<SapWeldConstraint<double>>(std::move(k_to_double));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapWeldConstraint);
