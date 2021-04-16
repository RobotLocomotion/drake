#pragma once

#include "drake/multibody/optimization/contact_wrench_evaluator.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace multibody {
/**
 * Formulates the nonlinear friction cone constraint |fₜ| ≤ μ*fₙ, where fₜ is
 * the tangential contact force, fₙ is the normal contact force, and μ is the
 * friction coefficient.
 *
 * The mathematical formulation of this constraint is
 *
 *     0 ≤ μ*fᵀn
 *     fᵀ((1+μ²)nnᵀ - I)f ≥ 0
 * where n is the unit length normal vector.
 * This formulation is equivalent to |fₜ| ≤ μ*fₙ, but the constraint is
 * differentiable everywhere (while |fₜ| ≤ μ*fₙ is not differentiable at fₜ =
 * 0.)
 *
 * The bound variables for this constraint is x = [q;λ], where q is the
 * generalized position, and λ is the parameterization of the contact wrench.
 *
 * @ingroup solver_evaluators
 */
class StaticFrictionConeConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StaticFrictionConeConstraint)

  /**
   * @param contact_wrench_evaluator. The evaluator takes in the generalized
   * position q, and a parameterization of the contact wrench λ, and evaluates
   * the contact wrench from body A to body B applied at the witness point of
   * geometry B expressed in the world frame, i.e., computes the contact wrench
   * F_Cb_W at the witness point p_WCb_W (see SignedDistancePair for the
   * definition of witness point), and F_Ca_W = -F_Cb_W (assuming Ca and Cb are
   * at the same spatial location).
   * @note although contact_wrench_evaluator computes both the contact force
   * and torque in the wrench, we only constrain the contact force to be
   * within the friction cone, and leave the torque unconstrained.
   */
  StaticFrictionConeConstraint(
      const ContactWrenchEvaluator* contact_wrench_evaluator);

  ~StaticFrictionConeConstraint() override {}

  /**
   * Given the bound variable @p x, decompose it into the generalized position
   * q, and λ as a parameterization of the contact wrench. x = [q; λ].
   */
  template <typename T>
  void DecomposeX(const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* q,
                  VectorX<T>* lambda) const {
    DRAKE_ASSERT(x.size() ==
                 contact_wrench_evaluator_->plant().num_positions() +
                     contact_wrench_evaluator_->num_lambda());
    *q = x.head(contact_wrench_evaluator_->plant().num_positions());
    *lambda = x.tail(contact_wrench_evaluator_->num_lambda());
  }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const final;
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const final;
  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const final;
  const ContactWrenchEvaluator* const contact_wrench_evaluator_;
};
}  // namespace multibody
}  // namespace drake
