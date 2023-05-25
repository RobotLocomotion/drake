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
    int objectA, RigidTransform<T> X_AP, RigidTransform<T> X_WA, int objectB,
    RigidTransform<T> X_BQ, RigidTransform<T> X_WB,
    SapConstraintJacobian<T> J_PQ_W)
    : objectA_(objectA),
      X_AP_(std::move(X_AP)),
      X_WA_(std::move(X_WA)),
      objectB_(objectB),
      X_BQ_(std::move(X_BQ)),
      X_WB_(std::move(X_WB)),
      J_(std::move(J_PQ_W)) {
  // Thus far only dense Jacobian blocks are supported, i.e. rigid body
  // applications in mind.
  DRAKE_THROW_UNLESS(J_.blocks_are_dense());

  const math::RigidTransform<T> X_WP = X_WA_ * X_AP_;
  const math::RigidTransform<T> X_WQ = X_WB_ * X_BQ_;
  X_WN_ = Interpolate(X_WP, X_WQ, T(0.5));

  const math::RigidTransform<T> X_NP = X_WN_.InvertAndCompose(X_WP);
  const math::RigidTransform<T> X_NQ = X_WN_.InvertAndCompose(X_WQ);
  X_NA_ = X_WN_.InvertAndCompose(X_WA_);
  X_NB_ = X_WN_.InvertAndCompose(X_WB_);

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

  // The constraint Jacobian is the relative spatial velocity between
  // P and Q expressed in frame N.
  // Then ġ = V_PQ_N = R_NW ⋅ V_PQ_W = R_NW ⋅ J_PQ_W ⋅ v.
  // Therefore the weld constraint Jacobian is J_PQ_N = R_NW * J_PQ_W.
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
  // Spatial impulse on B, applied at point No, expressed in frame N.
  const SpatialForce<T> gamma_BNo_N(gamma.template head<3>(),
                                    gamma.template tail<3>());
  const RotationMatrix<T>& R_WN = kinematics().X_WN().rotation();
  if (i == 0) {
    // Object A.
    // Shift to Ao and re-express in world.
    const Vector3<T>& p_NoAo_N = kinematics().X_NA().translation();
    *F += R_WN * (-gamma_BNo_N).Shift(p_NoAo_N);
  } else {
    // Object B.
    // Shift to Bo and re-express in world.
    const Vector3<T>& p_NoBo_N = kinematics().X_NB().translation();
    *F += R_WN * gamma_BNo_N.Shift(p_NoBo_N);
    return;
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapWeldConstraint)
