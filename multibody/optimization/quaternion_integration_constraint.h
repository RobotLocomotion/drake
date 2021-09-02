#pragma once

#include <limits>

#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
/**
 * If we have a body with orientation quaternion z₁ at time t₁, and a quaternion
 * z₂ at time t₂ = t₁ + h, with the angular velocity ω (expressed in the
 * world frame), we impose the constraint that the body rotates at a constant
 * velocity ω from quaternion z₁ to quaternion z₂ within time interval h. Namely
 * we want to enforce the relationship that z₂ and Δz⊗z₁ represent the same
 * orientation, where Δz is the quaternion [cos(|ω|h/2), ω/|ω|*sin(|ω|h/2)], and
 * ⊗ is the Hamiltonian product between quaternions.
 *
 * It is well-known that for any quaternion z, its element-wise negation -z
 * correspond to the same rotation matrix as z does. One way to undertand this
 * is that -z represents the rotation that first rotate the frame by a
 * quaternion z, and then continue to rotate about that axis for 360 degress. We
 * provide the option @p allow_quaternion_negation flag, that if set to
 * true, then we require that the quaternion z₂ = ±Δz⊗z₁. Otherwise we require
 * z₂ = Δz⊗z₁. Mathematically, the constraint we impose is
 *
 * If allow_quaternion_negation = true:
 *
 *     (z₂ • (Δz⊗z₁))² = 1
 *
 * else
 *
 *     z₂ • (Δz⊗z₁) = 1
 *
 * If your robot link orientation only changes slightly, and you are free to
 * search for both z₁ and z₂, then we would recommend to set
 * allow_quaternion_negation to false, as the left hand side of constraint z₂ •
 * (Δz⊗z₁) = 1 is less nonlinear than the left hand side of (z₂ • (Δz⊗z₁))² = 1.
 *
 * The operation • is the dot product between two quaternions, which computes
 * the cosine of the half angle between these two orientations. Dot product
 * equals to ±1 means that angle between the two quaternions are 2kπ, hence they
 * represent the same orientation.
 * @note The constraint is not differentiable at ω=0 (due to the
 * non-differentiability of |ω| at ω = 0). So it is better to initialize the
 * angular velocity to a non-zero value in the optimization.
 *
 * The decision variables of this constraint are [z₁, z₂, ω, h]
 *
 * @note We need to evaluate sin(|ω|h/2)/|ω|, when h is huge (larger than
 * 1/machine_epsilon), and |ω| is tiny (less than machine epsilon), this
 * evaluation is inaccurate. So don't use this constraint if you have a huge h
 * (which would be bad practice in trajectory optimization anyway).
 *
 * @ingroup solver_evaluators
 */
class QuaternionEulerIntegrationConstraint final : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuaternionEulerIntegrationConstraint)

  /**
   * @param allow_quaternion_negation. Refer to the class documentation.
   */
  explicit QuaternionEulerIntegrationConstraint(bool allow_quaternion_negation);

  ~QuaternionEulerIntegrationConstraint() override {}

  template <typename T>
  Eigen::Matrix<T, 12, 1> ComposeVariable(
      const Eigen::Ref<const Vector4<T>>& quat1,
      const Eigen::Ref<const Vector4<T>>& quat2,
      const Eigen::Ref<const Vector3<T>>& angular_vel, const T& h) const {
    Eigen::Matrix<T, 12, 1> vars;
    vars << quat1, quat2, angular_vel, h;
    return vars;
  }

  bool allow_quaternion_negation() const { return allow_quaternion_negation_; }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  bool allow_quaternion_negation_;
};

namespace internal {
// The 2-norm function |x| is not differentiable at x=0 (its gradient is x/|x|,
// which has a division-by-zero problem). On the other hand, x=0 happens very
// often. Hence we use a "smoothed" gradient as x/(|x| + ε) when x is almost 0.
template <typename T>
T DifferentiableNorm(const Vector3<T>& x) {
  const double kEps = std::numeric_limits<double>::epsilon();
  if constexpr (std::is_same_v<T, AutoDiffXd>) {
    const Eigen::Vector3d x_val = math::ExtractValue(x);
    const double norm_val = x_val.norm();
    if (norm_val > 100 * kEps) {
      return x.norm();
    } else {
      return AutoDiffXd(norm_val, math::ExtractGradient(x).transpose() * x_val /
                                      (norm_val + 10 * kEps));
    }
  } else {
    return x.norm();
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
