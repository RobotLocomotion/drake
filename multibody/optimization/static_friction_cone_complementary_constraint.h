#pragma once

#include "drake/multibody/optimization/contact_wrench_evaluator.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace multibody {
namespace internal {
/**
 * The nonlinear constraints to be imposed for static friction force. See
 * AddStaticFrictionConeComplementaryConstraint() for more details. The
 * nonlinear constraints are (1), (2), (4) and (6) in
 * AddStaticFrictionConeComplementaryConstraint()
 */
class StaticFrictionConeComplementaryNonlinearConstraint
    : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(
      StaticFrictionConeComplementaryNonlinearConstraint)

  /**
   * See AddStaticFrictionConeComplementaryConstraint for details.
   */
  StaticFrictionConeComplementaryNonlinearConstraint(
      const ContactWrenchEvaluator* contact_wrench_evaluator,
      double complementary_tolerance);

  ~StaticFrictionConeComplementaryNonlinearConstraint() override {}

  /** The slack variable for n_Wᵀ * f_W, see
   * AddStaticFrictionConeComplementaryConstraint().*/
  const symbolic::Variable& alpha_var() const { return alpha_var_; }

  /** The slack variable for sdf.see
   * AddStaticFrictionConeComplementaryConstraint(). */
  const symbolic::Variable& beta_var() const { return beta_var_; }

  void UpdateComplementaryTolerance(double complementary_tolerance);

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const final;
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const final;
  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const final;

  const ContactWrenchEvaluator* contact_wrench_evaluator_;

  symbolic::Variable alpha_var_;

  symbolic::Variable beta_var_;
};
};  // namespace internal

/** Adds the complementary constraint on the static friction force
 * between a pair of contacts
 * |ft_W| <= μ * n_Wᵀ * f_W  (static friction force in the friction cone).
 * fn_W * sdf = 0 (complementary condition)
 * sdf >= 0 (no penetration)
 * where sdf stands for signed distance function, ft_W stands for the tangential
 * friction force expressed in the world frame.
 *
 * Mathematically, we impose the following constraint
 * f_Wᵀ * (I - nnᵀ) * f_W <= μ² * fᵀ * n * nᵀ * f   (1)
 * n_Wᵀ * f_W = α                                   (2)
 * α >= 0                                           (3)
 * sdf = β                                          (4)
 * β >= 0                                           (5)
 * α * β <= ε                                       (6)
 * @param contact_wrench_evaluator The evaluator to compute the contact wrench
 * expressed in the world frame.
 * @param q_vars The decision variable for the generalized configuration q.
 * @param lambda_vars The decision variable to parameterize the contact wrench.
 * @param[in/out] prog The optimization program to which the constraint is
 * added.
 * @param complementary_tolerance ε in the documentation above.
 * @return binding The binding containing the nonlinear constraints
 * (1)(2)(4)(6).
 * @pre `q_vars` and `lambda_vars` have been added to `prog` before calling this
 * function.
 */
solvers::Binding<internal::StaticFrictionConeComplementaryNonlinearConstraint>
AddStaticFrictionConeComplementaryConstraint(
    const ContactWrenchEvaluator* contact_wrench_evaluator,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& lambda_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    double complementary_tolerance, solvers::MathematicalProgram* prog);

}  // namespace multibody
}  // namespace drake
