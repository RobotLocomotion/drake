#include "drake/multibody/optimization/quaternion_integration_constraint.h"

#include <limits>

#include "drake/common/extract_double.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace multibody {
const double kEps = std::numeric_limits<double>::epsilon();

namespace {
// The 2-norm function |x| is not differentiable at x=0 (its gradient is x/|x|,
// which has a division-by-zero problem). On the other hand, x=0 happens very
// often. Hence we use a "smoothed" gradient as x/(|x| + ε) when x is almost 0.
template <typename T>
T DifferentiableNorm(const Vector3<T>& x) {
  if constexpr (std::is_same_v<T, AutoDiffXd>) {
    const Eigen::Vector3d x_val = math::autoDiffToValueMatrix(x);
    const double norm_val = x_val.norm();
    if (norm_val > 100 * kEps) {
      return x.norm();
    } else {
      return AutoDiffXd(norm_val,
                        math::autoDiffToGradientMatrix(x).transpose() * x_val /
                            (norm_val + 10 * kEps));
    }
  } else {
    return x.norm();
  }
}

// Evaluate the left-hand side of the constraint
// dot_product(z₂⊗ z₁*, Δz) = 1
// where Δz = [cos(|ω|h/2), ω/(|ω|)sin(|ω|h/2)]
template <typename T>
Vector1<T> EvalQuaternionIntegration(const Eigen::Quaternion<T>& quat1,
                                     const Eigen::Quaternion<T>& quat2,
                                     const Vector3<T>& angular_vel,
                                     const T& h) {
  const Eigen::Quaternion<T> quat2_times_quat1_conj =
      quat2 * (quat1.conjugate());
  Vector1<T> ret;
  using std::cos;
  using std::sin;
  // Now compute sin(|ω|h/2)/|ω|
  // when ω = 0, sin(|ω|h/2)/|ω|=h/2 (By taking the limit as |ω|-> 0)
  // otherwise, we compute sin(|ω|h/2)/|ω| directly.
  const T angular_vel_norm = DifferentiableNorm(angular_vel);
  Vector3<T> delta_z_vec;
  if (ExtractDoubleOrThrow(angular_vel_norm) == 0) {
    delta_z_vec = angular_vel * h / 2;
  } else {
    delta_z_vec =
        angular_vel * sin(angular_vel_norm * h / 2) / angular_vel_norm;
  }
  if constexpr (std::is_same_v<T, double>) {
  }
  const T delta_z_w = cos(angular_vel_norm * h / 2);
  const Eigen::Quaternion<T> delta_z(delta_z_w, delta_z_vec(0), delta_z_vec(1),
                                     delta_z_vec(2));
  ret(0) = quat2_times_quat1_conj.dot(delta_z);
  return ret;
}

template <typename T>
void EulerIntegrationEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                                 VectorX<T>* y) {
  const Eigen::Quaternion<T> quat1(x(0), x(1), x(2), x(3));
  const Eigen::Quaternion<T> quat2(x(4), x(5), x(6), x(7));
  const Vector3<T> angular_vel(x.template segment<3>(8));
  *y = EvalQuaternionIntegration<T>(quat1, quat2, angular_vel, x(11));
}
}  // namespace

QuaternionEulerIntegrationConstraint::QuaternionEulerIntegrationConstraint()
    : solvers::Constraint(1, 12, Vector1d::Ones(), Vector1d::Ones()) {
  this->set_description("quaternion Euler integration constraint");
}

void QuaternionEulerIntegrationConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  EulerIntegrationEvalGeneric<double>(x, y);
}

void QuaternionEulerIntegrationConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  EulerIntegrationEvalGeneric<AutoDiffXd>(x, y);
}

void QuaternionEulerIntegrationConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  EulerIntegrationEvalGeneric<symbolic::Expression>(
      x.cast<symbolic::Expression>(), y);
}
}  // namespace multibody
}  // namespace drake
