#pragma once

#include <memory>
#include <vector>

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
 * The bound variables for this constraint is x = [q; λ; α; β]
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

  const ContactWrenchEvaluator& contact_wrench_evaluator() const {
    return *contact_wrench_evaluator_;
  }

  template <typename T>
  void DecomposeX(const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* q,
                  VectorX<T>* lambda, T* alpha, T* beta) const {
    *q = x.head(contact_wrench_evaluator_->plant().num_positions());
    *lambda = x.segment(q->rows(), contact_wrench_evaluator_->num_lambda());
    *alpha = x(x.rows() - 2);
    *beta = x(x.rows() - 1);
  }

  /**
   * Create a binding of the constraint, together with the bound variables
   * x = [q; λ; α; β]. See StaticFrictionConeComplementaryNonlinearConstraint()
   * constructor for more details.
   */
  static solvers::Binding<
      internal::StaticFrictionConeComplementaryNonlinearConstraint>
  MakeBinding(const ContactWrenchEvaluator* contact_wrench_evaluator,
              double complementary_tolerance,
              const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
              const Eigen::Ref<const VectorX<symbolic::Variable>>& lambda_vars);

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const final;
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const final;
  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const final;

  const ContactWrenchEvaluator* const contact_wrench_evaluator_;

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
 * Mathematically, we add the following constraints to the optimization program
 * f_Wᵀ * ((μ² + 1)* n_W * n_Wᵀ - I) * f_W >= 0                    (1)
 * n_Wᵀ * f_W = α                                                  (2)
 * α >= 0                                                          (3)
 * sdf = β                                                         (4)
 * β >= 0                                                          (5)
 * 0 <= α * β <= ε                                                 (6)
 * the slack variables α and β are added to the optimization program as well.
 *
 * @param contact_wrench_evaluators_and_lambda The evaluators to compute the
 * contact wrench expressed in the world frame, together with λ used to
 * parameterize the contact wrench.
 * @param q_vars The decision variable for the generalized configuration q.
 * @param lambda_vars The decision variable to parameterize the contact wrench.
 * @param[in/out] prog The optimization program to which the constraint is
 * added.
 * @param complementary_tolerance ε in the documentation above.
 * @return binding The binding containing the nonlinear constraints
 * (1)(2)(4)(6).
 * @pre Both `q_vars` and `lambda_vars` have been added to `prog` before calling
 * this function.
 */
solvers::Binding<internal::StaticFrictionConeComplementaryNonlinearConstraint>
AddStaticFrictionConeComplementaryConstraint(
    const ContactWrenchEvaluator* contact_wrench_evaluator,
    double complementary_tolerance,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& lambda_vars,
    solvers::MathematicalProgram* prog);

}  // namespace multibody
}  // namespace drake
