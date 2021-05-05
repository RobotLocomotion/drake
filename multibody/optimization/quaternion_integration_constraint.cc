#include "drake/multibody/optimization/quaternion_integration_constraint.h"

#include <limits>

#include "drake/common/extract_double.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace multibody {

namespace {
// If allow_quaternion_negation = true, evaluate the left-hand side of the
// constraint
//     (z₂ • (Δz⊗z₁))² = 1
// else, evaluate the left-hand side of the constraint
//     z₂ • (Δz⊗z₁) = 1
// where Δz = [cos(|ω|h/2), ω/(|ω|)sin(|ω|h/2)]
template <typename T>
Vector1<T> EvalQuaternionIntegration(const Eigen::Quaternion<T>& quat1,
                                     const Eigen::Quaternion<T>& quat2,
                                     const Vector3<T>& angular_vel, const T& h,
                                     bool allow_quaternion_negation) {
  Vector1<T> ret;
  using std::cos;
  using std::sin;
  // Now compute sin(|ω|h/2)/|ω|
  // when ω = 0, sin(|ω|h/2)/|ω|=h/2 (By taking the limit as |ω|-> 0)
  // otherwise, we compute sin(|ω|h/2)/|ω| directly.
  const T angular_vel_norm = internal::DifferentiableNorm(angular_vel);
  // The "vector" part of Δz.
  Vector3<T> delta_z_vec;
  if (scalar_predicate<T>::is_bool &&
      ExtractDoubleOrThrow(angular_vel_norm) == 0) {
    delta_z_vec = angular_vel * h / 2;
  } else {
    delta_z_vec =
        angular_vel * sin(angular_vel_norm * h / 2) / angular_vel_norm;
  }
  const T delta_z_w = cos(angular_vel_norm * h / 2);
  const Eigen::Quaternion<T> delta_z(delta_z_w, delta_z_vec(0), delta_z_vec(1),
                                     delta_z_vec(2));
  const Eigen::Quaternion<T> delta_z_times_quat1 = delta_z * quat1;
  if (allow_quaternion_negation) {
    using std::pow;
    ret(0) = pow(quat2.dot(delta_z_times_quat1), 2);
  } else {
    ret(0) = quat2.dot(delta_z_times_quat1);
  }
  return ret;
}

template <typename T>
void EulerIntegrationEvalGeneric(bool allow_quaternion_negation,
                                 const Eigen::Ref<const VectorX<T>>& x,
                                 VectorX<T>* y) {
  const Eigen::Quaternion<T> quat1(x(0), x(1), x(2), x(3));
  const Eigen::Quaternion<T> quat2(x(4), x(5), x(6), x(7));
  const Vector3<T> angular_vel(x.template segment<3>(8));
  *y = EvalQuaternionIntegration<T>(quat1, quat2, angular_vel, x(11),
                                    allow_quaternion_negation);
}
}  // namespace

QuaternionEulerIntegrationConstraint::QuaternionEulerIntegrationConstraint(
    bool allow_quaternion_negation)
    : solvers::Constraint(1, 12, Vector1d::Ones(), Vector1d::Ones()),
      allow_quaternion_negation_{allow_quaternion_negation} {
  this->set_description("quaternion Euler integration constraint");
}

void QuaternionEulerIntegrationConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  EulerIntegrationEvalGeneric<double>(allow_quaternion_negation_, x, y);
}

void QuaternionEulerIntegrationConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  EulerIntegrationEvalGeneric<AutoDiffXd>(allow_quaternion_negation_, x, y);
}

void QuaternionEulerIntegrationConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  EulerIntegrationEvalGeneric<symbolic::Expression>(
      allow_quaternion_negation_, x.cast<symbolic::Expression>(), y);
}
}  // namespace multibody
}  // namespace drake
