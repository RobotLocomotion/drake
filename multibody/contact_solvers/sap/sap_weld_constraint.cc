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

namespace {
// Returns the pose of a frame F in frame M, where the pose of F is defined by
// interpolating between the poses of frame A and B in frame M. The position of
// the origin of F is given by linear interpolation between p_MAo and p_MBo:
//   p_MFo = (1 - t) * p_MAo + t * p_MBo
// and the orientation of F in M:
//   R_MF = slerp(R_MA, R_MB, t)
// We take the same convention as Eigen's slerp function in that:
//   Interpolate(X_MA, X_MB, 0) = X_MA and Interpolate(X_MA, X_MB, 1) = X_MB
// Only valid for 0 <= t <= 1 and will throw an exception otherwise.
template <typename T>
RigidTransform<T> Interpolate(const RigidTransform<T>& X_MA,
                              const RigidTransform<T> X_MB, const T& t) {
  DRAKE_THROW_UNLESS(0 <= t && t <= 1);
  return RigidTransform<T>(
      X_MA.rotation().ToQuaternion().slerp(t, X_MB.rotation().ToQuaternion()),
      (1.0 - t) * X_MA.translation() + t * X_MB.translation());
}
}  // namespace

template <typename T>
SapWeldConstraint<T>::Kinematics::Kinematics(
    int objectA, RigidTransform<T> X_WP, Vector3<T> p_AP_W, int objectB,
    RigidTransform<T> X_WQ, Vector3<T> p_BQ_W, SapConstraintJacobian<T> J_PQ_W)
    : objectA_(objectA),
      X_WP_(std::move(X_WP)),
      p_AP_W_(std::move(p_AP_W)),
      objectB_(objectB),
      X_WQ_(std::move(X_WQ)),
      p_BQ_W_(std::move(p_BQ_W)),
      J_(std::move(J_PQ_W)) {
  // Thus far only dense Jacobian blocks are supported, i.e. rigid body
  // applications in mind.
  DRAKE_THROW_UNLESS(J_.blocks_are_dense());

  X_WN_ = Interpolate(X_WP_, X_WQ_, T(0.5));

  const math::RigidTransform<T> X_NP = X_WN_.InvertAndCompose(X_WP);
  const math::RigidTransform<T> X_NQ = X_WN_.InvertAndCompose(X_WQ);

  p_PoQo_N_ = X_NQ.translation() - X_NP.translation();
  const RotationMatrix<T> R_PQ = X_NP.rotation().transpose() * X_NQ.rotation();

  const Matrix3<T>& R = R_PQ.matrix();

  // Rodrigues' formula states that for a rotation matrix R:
  //   R = I + sin(θ)⋅K + (1−cos(θ))⋅K²
  // where K = k× is the skew-symmetric cross product matrix
  // (such that k×⋅v = k×v), and where θ and k are the angle and axis
  // representation of rotation matrix R. Because I and K² are symmetric
  // and K is antisymmetric, we can write:
  //   R - Rᵀ = 2⋅sin(θ)⋅K
  // For small angles, we can approximate sin(θ) ≈ θ.
  // Using this approximation, we can write sin(θ)⋅K ≈ θ⋅K = a×, where we
  // defined a = θk. We can then recover the components of a using
  //   a× = (R-Rᵀ)/2
  // and the definition of the skew-symmetric cross product matrix:
  //
  //        |   0  -a₂  a₁ |
  //   a× = |  a₂   0  -a₀ |
  //        | -a₁  a₀   0  |
  //
  a_PQ_N_ = Vector3<T>(0.5 * (R(2, 1) - R(1, 2)), 0.5 * (R(0, 2) - R(2, 0)),
                       0.5 * (R(1, 0) - R(0, 1)));
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
      (Vector6<T>() << kinematics.a_PQ_N(), kinematics.p_PoQo_N()).finished();
  Vector6<T> b = Vector6<T>::Zero();  // Bias term.

  // Consider the cosntraint velocity ġ = (ȧ_PQ_N, ṗ_PoQo_N). The tranlational
  // component:
  //   ṗ_PoQo_N = v_M_PoQo_N
  // I.e. the velocity of Qo relative to Po, measured in some frame M and
  // expressed in N. Only when P and Q are coincident is their relative velocity
  // independent of which frame it is measured in. This means that the way we
  // model this constraint lacks frame-invariance. It is unclear at this time if
  // choosing a particular frame leads to a better approximation, conditioning,
  // etc. We choose M = W, the world frame, for convenience. The angular
  // component:
  //   ȧ_PQ_N = d(θk)/dt = dθ/dt⋅k + θ⋅dk/dt
  // From [Mitiguy 2016, §9.2.2]:
  //   θ⋅dk/dt = θ/2⋅[(cos(θ/2)/sin(θ/2))⋅(ᴬωᴮ − dθ/dt⋅k) + k×ᴬωᴮ)]
  // We make a small angle assumption θ ≈ 0 to get:
  //   θ⋅dk/dt ≈ ᴬωᴮ − dθ/dt⋅k + θ/2⋅k×ᴬωᴮ
  // Thus:
  //   ȧ_PQ_N ≈ dθ/dt⋅k + ᴬωᴮ − dθ/dt⋅k + θ/2⋅k×ᴬωᴮ
  //          = ᴬωᴮ + θ/2⋅k×ᴬωᴮ
  // Again making the small angle assumption and ignoring the θ dependent term:
  //   ȧ_PQ_N ≈ ᴬωᴮ
  // Then:
  //   ġ = (ȧ_PQ_N, ṗ_PoQo_N) = (ᴬωᴮ, v_W_PoQo_N) = V_W_PQ_N
  //
  // Thus the constraint Jacobian is the relative spatial velocity jacobian
  // between P and Q expressed in frame N:
  //  ġ = V_PQ_N = R_NW ⋅ V_PQ_W = (R_NW ⋅ J_PQ_W) ⋅ v = J ⋅ v
  const MatrixX<T> R_NW = kinematics.X_WN().rotation().matrix().transpose();

  const SapConstraintJacobian<T>& J = kinematics.jacobian();
  const MatrixX<T> J_PQ_W_first_clique = J.clique_jacobian(0).MakeDenseMatrix();
  MatrixX<T> J_PQ_N_first_clique(J_PQ_W_first_clique.rows(),
                                 J_PQ_W_first_clique.cols());
  J_PQ_N_first_clique << R_NW * J_PQ_W_first_clique.template topRows<3>(),
      R_NW * J_PQ_W_first_clique.template bottomRows<3>();

  if (kinematics.jacobian().num_cliques() == 1) {
    return typename SapHolonomicConstraint<T>::Kinematics(
        std::move(g),
        SapConstraintJacobian<T>(J.clique(0), std::move(J_PQ_N_first_clique)),
        std::move(b));
  }

  const MatrixX<T> J_PQ_W_second_clique =
      J.clique_jacobian(1).MakeDenseMatrix();
  MatrixX<T> J_PQ_N_second_clique(J_PQ_W_second_clique.rows(),
                                  J_PQ_W_second_clique.cols());
  J_PQ_N_second_clique << R_NW * J_PQ_W_second_clique.template topRows<3>(),
      R_NW * J_PQ_W_second_clique.template bottomRows<3>();

  return typename SapHolonomicConstraint<T>::Kinematics(
      std::move(g),
      SapConstraintJacobian<T>(J.clique(0), std::move(J_PQ_N_first_clique),
                               J.clique(1), std::move(J_PQ_N_second_clique)),
      std::move(b));
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
  // We know that J⋅v = V_W_PQ_N = V_WQ_N - V_WP_N. Thus J is just the operator
  // that takes the difference of the body's spatial velocities measured and
  // expressed in world, shifts to P and Q and re-expresses in N:
  //   J = [         -[R_NW]         0            [R_NW]       0]
  //       [[R_NW]⋅[p_AP_W]ₓ  -[R_NW]  -[R_NW]⋅[p_BQ_W]ₓ [R_NW]]
  // We also know from the optimality condition of the SAP formulation
  // (essentially the balance of momentum condition):
  //   A⋅(v - v*) - Jᵀ⋅γ = 0
  // Therefore the generalized impulse Jᵀ⋅γ corresponds to a spatial impulse on
  // body A and a spatial impulse on body B stacked:
  //   Jᵀ⋅γ = [Γ_Ao_W, Γ_Bo_W]
  // Where:
  //   Γ_Ao_W = ([R_WN]⋅(-γᵣ) - [p_PA]ₓ⋅[R_WN]⋅(-γₜ), [R_WN]⋅(-γₜ)) and
  //   Γ_Bo_W = ([R_WN]⋅( γᵣ) - [p_QB]ₓ⋅[R_WN]⋅( γᵣ), [R_WN]⋅( γₜ))
  // Therefore Jᵀ can be understood as the operator that takes a spatial impulse
  // -γ (expressed in N) applied on body A at Po, re-expresses in W, and shifts
  // to Ao. As well it takes the equal and opposite spatial impulse γ (expressed
  // in N) applied on body B at Qo, re-expresses in W, and shifts to Bo.
  // Thus, this constraint can be interpreted as applying an impulse γ at point
  // Qo on B and an impulse -γ at point Po on A. As a consequence the constraint
  // satisfies Newton's 3rd law, but does introduce a small moment of order
  // O(‖γ‖⋅‖p_PQ‖) when P and Q are not coincident.

  // Spatial impulse on B, applied at point Qo, expressed in frame N.
  const SpatialForce<T> gamma_BQo_N(gamma.template head<3>(),
                                    gamma.template tail<3>());
  const RotationMatrix<T>& R_WN = kinematics().X_WN().rotation();
  if (i == 0) {
    // Object A.
    // Re-express in world and shift to Ao.
    // -gamma_BQo_N = gamma_APo_N.
    *F += (R_WN * -gamma_BQo_N).Shift(-kinematics().p_AP_W());
  } else {
    // Object B.
    // Re-express in world and shift to Bo.
    *F += (R_WN * gamma_BQo_N).Shift(-kinematics().p_BQ_W());
    return;
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapWeldConstraint)
