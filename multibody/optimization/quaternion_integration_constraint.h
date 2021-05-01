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
 *     If allow_quaternion_negation = true:
 *     (z₂ • (Δz⊗z₁))² = 1
 *     else
 *     z₂ • (Δz⊗z₁) = 1
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
 * @ingroup solver_evaluators
 */
class QuaternionEulerIntegrationConstraint final : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuaternionEulerIntegrationConstraint)

  /**
   * @param allow_quaternion_negation. Refer to the class documentation. If set
   * to true, then we regard a quaternion z and its elementwise negation -z to
   * represent the same orientation, and impose the constraint
   *
   *     ((z₂⊗z₁*) • Δz)² = 1
   *
   * otherwise, we impose the constraint
   *
   *     (z₂⊗z₁*) • Δz = 1
   */
  explicit QuaternionEulerIntegrationConstraint(
      bool negate_quaternion_same_orientation);

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
}  // namespace multibody
}  // namespace drake
