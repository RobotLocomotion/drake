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
template <typename T>
RigidTransform<T> Interpolate(const RigidTransform<T>& X_MA,
                              const RigidTransform<T> X_MB, const T& t) {
  return RigidTransform<T>(
      X_MA.rotation().ToQuaternion().slerp(t, X_MB.rotation().ToQuaternion()),
      (1.0 - t) * X_MA.translation() + t * X_MB.translation());
}
}  // namespace

template <typename T>
SapWeldConstraint<T>::Kinematics::Kinematics(
    int objectA, RigidTransform<T> X_AP, RigidTransform<T> X_WA, int objectB,
    RigidTransform<T> X_BQ, RigidTransform<T> X_WB,
    SapConstraintJacobian<T> Jv_PQ_W)
    : objectA_(objectA),
      X_AP_(std::move(X_AP)),
      X_WA_(std::move(X_WA)),
      objectB_(objectB),
      X_BQ_(std::move(X_BQ)),
      X_WB_(std::move(X_WB)),
      J_(std::move(Jv_PQ_W)) {
  // Thus far only dense Jacobian blocks are supported, i.e. rigid body
  // applications in mind.
  DRAKE_THROW_UNLESS(J_.blocks_are_dense());

  const math::RigidTransform<T>& X_WP = X_WA_ * X_AP_;
  const math::RigidTransform<T>& X_WQ = X_WB_ * X_BQ_;
  X_WN_ = Interpolate(X_WP, X_WQ, T(0.5));

  const math::RigidTransform<T>& X_NP = X_WN_.InvertAndCompose(X_WP);
  const math::RigidTransform<T>& X_NQ = X_WN_.InvertAndCompose(X_WQ);
  X_NA_ = X_WN_.InvertAndCompose(X_WA_);
  X_NB_ = X_WN_.InvertAndCompose(X_WB_);

  p_PoQo_N_ = X_NQ.translation() - X_NP.translation();
  const RotationMatrix<T> R_PQ = X_NP.rotation().transpose() * X_NQ.rotation();

  const Matrix3<T>& R = R_PQ.matrix();
  a_N_ = Vector3<T>(0.5 * (R(2, 1) - R(1, 2)), 0.5 * (R(0, 2) - R(2, 0)),
                    0.5 * (R(1, 0) - R(0, 1)));
}

template <typename T>
SapWeldConstraint<T>::SapWeldConstraint(Kinematics kinematics,
                                        const T& relaxation_time)
    : SapHolonomicConstraint<T>(
          MakeSapHolonomicConstraintKinematics(kinematics),
          MakeSapHolonomicConstraintParameters(relaxation_time),
          {kinematics.objectA(), kinematics.objectB()}),
      kinematics_(std::move(kinematics)) {}

template <typename T>
typename SapHolonomicConstraint<T>::Parameters
SapWeldConstraint<T>::MakeSapHolonomicConstraintParameters(
    const T& relaxation_time) {
  // "Near-rigid" regime parameter, see [Castro et al., 2022].
  // TODO(amcastro-tri): consider exposing this parameter.
  constexpr double kBeta = 0.1;

  // Weld constraints do not have impulse limits, they are bi-lateral
  // constraints. Each weld constraint introduces a single constraint
  // equation.
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const Vector6<T> infinity = kInf * Vector6<T>::Ones();
  return typename SapHolonomicConstraint<T>::Parameters{
      -infinity, infinity, infinity, relaxation_time * Vector6<T>::Ones(),
      kBeta};
}

template <typename T>
typename SapHolonomicConstraint<T>::Kinematics
SapWeldConstraint<T>::MakeSapHolonomicConstraintKinematics(
    const Kinematics& kinematics) {
  // Constraint function.
  Vector6<T> g =
      (Vector6<T>() << kinematics.a_N(), kinematics.p_PoQo_N()).finished();
  Vector6<T> b = Vector6<T>::Zero();  // Bias term.

  // The constraint Jacobian is the relative spatial velocity between
  // P and Q expressed in frame N. Let R be the 6x6 matrix that re-expresses
  // spatial velocity V_W in frame N.
  // That is: R ⋅ V_W = V_N, where  R = [ R_WN     0 ]
  //                                    [    0  R_NW ]
  // Then ġ = V_PQ_N = R ⋅ V_PQ_W = R ⋅ Jv_PQ_W ⋅ v.
  // Therefore the weld constraint Jacobian is Jweld = R * Jv_PQ_W.
  const MatrixX<T>& R_NW = kinematics.X_WN().rotation().matrix().transpose();

  const SapConstraintJacobian<T> J = kinematics.jacobian();
  const MatrixX<T> JA_W = J.clique_jacobian(0).MakeDenseMatrix();
  MatrixX<T> JA_N(JA_W.rows(), JA_W.cols());
  JA_N << R_NW * JA_W.template topRows<3>(),
      R_NW * JA_W.template bottomRows<3>();

  if (kinematics.jacobian().num_cliques() == 1) {
    return typename SapHolonomicConstraint<T>::Kinematics(
        std::move(g), SapConstraintJacobian<T>(J.clique(0), std::move(JA_N)),
        std::move(b));
  }

  const MatrixX<T> JB_W = J.clique_jacobian(1).MakeDenseMatrix();
  MatrixX<T> JB_N(JB_W.rows(), JB_W.cols());
  JB_N << R_NW * JB_W.template topRows<3>(),
      R_NW * JB_W.template bottomRows<3>();

  return typename SapHolonomicConstraint<T>::Kinematics(
      std::move(g),
      SapConstraintJacobian<T>(J.clique(0), std::move(JA_N), J.clique(1),
                               std::move(JB_N)),
      std::move(b));
}

template <typename T>
void SapWeldConstraint<T>::DoAccumulateSpatialImpulses(
    int i, const Eigen::Ref<const VectorX<T>>& gamma,
    SpatialForce<T>* F) const {
  // gamma is F_No_N
  SpatialForce<T> F_No_N(gamma.template segment<3>(0),
                         gamma.template segment<3>(3));
  if (i == 0) {
    // Object A.
    // Shift to Ao and re-express in world.
    SpatialForce<T> F_Ao_N = -F_No_N.Shift(kinematics_.X_NA().translation());
    *F +=
        SpatialForce<T>(kinematics_.X_WN().rotation() * F_Ao_N.rotational(),
                        kinematics_.X_WN().rotation() * F_Ao_N.translational());
  } else {
    // Object B.
    // Shift to Bo and re-express in world.
    SpatialForce<T> F_Bo_N = F_No_N.Shift(kinematics_.X_NB().translation());
    *F +=
        SpatialForce<T>(kinematics_.X_WN().rotation() * F_Bo_N.rotational(),
                        kinematics_.X_WN().rotation() * F_Bo_N.translational());
    return;
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapWeldConstraint)
