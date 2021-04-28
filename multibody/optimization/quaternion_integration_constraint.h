#pragma once

#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
/**
 * If we have a body with orientation quaternion z₁ at time t₁, and a quaternion
 * z₂ at time t₂ = t₁ + h, with the angular velocity ω (expressed in the
 * world frame). We impose the constraint that the body rotates at a constant
 * velocity ω from quaternion z₁ to quaternion z₂ within time interval h. Namely
 * we want to enforce the relationship that z₂ and Δz⊗z₁ represent the same
 * orientation, where Δz is the quaternion [cos(|ω|h/2), ω/|ω|*sin(|ω|h/2)], and
 * ⊗ is the Hamiltonian product between quaternions. Notice that Δz contains the
 * term ω/|ω|, which could cause division-by-zero problem. To avoid this
 * problem, we replace the term ω/|ω| with ω/(|ω|+ε) where ε is a very small
 * number.
 *
 * Mathematically, the constraint we impose is
 *
 *     (z₂⊗z₁*) • Δz = 1
 *
 * where z₁* is the conjugate of the quaternion z₁. The term z₂⊗ z₁* computes
 * the unit quaternion representing the orientation difference between z₁ and
 * z₂.
 * The operation • is the dot product between two quaternions, which computes
 * the cosine of the half angle between these two orientations. Hence dot
 * product equals to 1 means the two quaternions represent the same orientation.
 * @note The constraint is not differentiable at ω=0 (due to the
 * non-differentiability of |ω| at ω = 0). So it is better to initialize the
 * angular velocity to a non-zero value in the optimization.
 *
 * The decision variables of this constraint are [z₁, z₂, ω, h]
 *
 * @ingroup solver_evaluators
 */
class QuaternionEulerIntegrationConstraint final : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuaternionEulerIntegrationConstraint)

  QuaternionEulerIntegrationConstraint();

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

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;
};
}  // namespace multibody
}  // namespace drake
