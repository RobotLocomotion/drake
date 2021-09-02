#pragma once

#include <utility>
#include <vector>

#include "drake/multibody/optimization/contact_wrench_evaluator.h"
#include "drake/multibody/optimization/static_friction_cone_complementarity_constraint.h"
#include "drake/multibody/optimization/static_friction_cone_constraint.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"

/** @file
 * @anchor sliding_friction_complementarity_constraint
 * Impose the complementarity constraint on the sliding friction using a Coulomb
 * friction cone model. If the contact is sliding, then the tangential
 * friction force is in the opposite direction to the sliding velocity, and the
 * contact force is at the boundary of the friction cone. If the contact is
 * static, then the contact force is within the friction cone.
 *
 * We decompose the contact force f into two parts, the static contact force
 * f_static, and the contact force during sliding f_sliding. Namely
 *
 *     f = f_static + f_sliding                           (1)
 *
 * In order to enforce the constraint that when the contact is sliding, the
 * static friction force is 0, we impose the complementarity constraint
 *
 *     v_sliding_tangential * f_static_normal = 0         (2)
 *
 * To enforce that the sliding force is on the boundary of the friction cone,
 * we impose
 *
 *     μ * f_sliding_normal = |f_sliding_tangential|      (3)
 *
 * To enforce that the sliding force is in the opposite direction of the sliding
 * velocity, we impose the following constraint with a slack variable c.
 *
 *     f_sliding_tangential = -c * v_sliding_tangential   (4)
 *     c ≥ 0                                              (5)
 *
 * To enforce that the contact force is in the friction cone, we impose
 *
 *     | f_tangential | ≤ μ f_normal                      (6)
 *
 * Depending on whether the contact is explicit or implicit, we add constraint
 * (6) by either calling
 * AddSlidingFrictionComplementarityExplicitContactConstraint(), if the user
 * knows explicitly that the contact occurs, or calling
 * AddSlidingFrictionComplementarityImplicitContactConstraint() if the user
 * doesn't know if the contact has to occur.
 */
namespace drake {
namespace multibody {
namespace internal {
/*
 * Impose the nonlinear constraints in @ref
 * sliding_friction_complementarity_constraint, namely constraint (1) - (4).
 * Notice that we will relax the complementarity constraint (2) as
 *
 *     -ε ≤ v_slidingᵀ * f_static ≤ ε
 *
 * with a small non-negative constant ε.
 * The bound variable vector for this constraint is [q; v; λ; f_static;
 * f_sliding; c].
 */
class SlidingFrictionComplementarityNonlinearConstraint
    : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(
      SlidingFrictionComplementarityNonlinearConstraint)

  /*
   * @param contact_wrench_evaluator An evaluator that computes the contact
   * wrench between a pair of geometries. We will only impose the constraint on
   * the contact force, the contact torque is unconstrained.
   * @param complementarity_tolerance The small constant ε for relaxing the
   * complementarity constraint (2).
   */
  SlidingFrictionComplementarityNonlinearConstraint(
      const ContactWrenchEvaluator* contact_wrench_evaluator,
      double complementarity_tolerance);

  ~SlidingFrictionComplementarityNonlinearConstraint() override {}

  void UpdateComplementarityTolerance(double complementarity_tolerance);

  /*
   * Getter for the slack variable c, used in the constraint
   *
   *     f_sliding_tangential = -c * v_sliding
   */
  const symbolic::Variable& c_var() const { return c_var_; }

  template <typename T>
  void DecomposeX(const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* q,
                  VectorX<T>* v, VectorX<T>* lambda, Vector3<T>* f_static,
                  Vector3<T>* f_sliding, T* c) const {
    *q = x.head(contact_wrench_evaluator_->plant().num_positions());
    *v = x.segment(q->rows(),
                   contact_wrench_evaluator_->plant().num_velocities());
    *lambda = x.segment(q->rows() + v->rows(),
                        contact_wrench_evaluator_->num_lambda());
    *f_static = x.template segment<3>(q->rows() + v->rows() + lambda->rows());
    *f_sliding =
        x.template segment<3>(q->rows() + v->rows() + lambda->rows() + 3);
    *c = x(x.rows() - 1);
  }

  template <typename T>
  void ComposeX(const Eigen::Ref<const VectorX<T>>& q,
                const Eigen::Ref<const VectorX<T>>& v,
                const Eigen::Ref<const VectorX<T>>& lambda,
                const Vector3<T>& f_static, const Vector3<T>& f_sliding,
                const T& c, VectorX<T>* x) const {
    x->resize(num_vars());
    *x << q, v, lambda, f_static, f_sliding, c;
  }

  /*
   * Return the sparsity pattern of the constraint, when we compute the gradient
   * of the constraint w.r.t the variable itself (namely when
   * ExtractGradient(x) = Identity in DoEval(x, y)).
   * @return gradient_sparsity_pattern. The pairs
   * (gradient_sparsity_pattern[i].first, gradient_sparsity_pattern[i].second)
   * contain all the (row, column) index pairs of the non-zero entries in the
   * gradient.
   */
  std::vector<std::pair<int, int>> GetConstraintSparsityPattern() const;

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const final;
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const final;
  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const final;

  const ContactWrenchEvaluator* const contact_wrench_evaluator_;
  symbolic::Variable c_var_;
};
}  // namespace internal

/**
 * For a pair of geometries in explicit contact, adds the sliding friction
 * complementarity constraint explained in @ref
 * sliding_friction_complementarity_constraint to an optimization program. This
 * function adds the slack variables (f_static, f_sliding, c), and impose all
 * the constraints in @ref sliding_friction_complementarity_constraint.
 * @param contact_wrench_evaluator Evaluates the contact wrench between a pair
 * of geometries.
 * @param complementarity_tolerance The tolerance on the complementarity
 * constraint.
 * @param q_vars The variable for the generalized position q in @p prog.
 * @param v_vars The variable for the generalized velocity v in @p prog.
 * @param lambda_vars The variables to parameterize the contact wrench between
 * this pair of geometry.
 * @param prog The optimization program to which the sliding friction
 * complementarity constraint is imposed.
 * @return (sliding_friction_complementarity_constraint,
 * static_friction_cone_constraint), the pair of constraint that imposes (1)-(4)
 * and (6) in @ref sliding_friction_complementarity_constraint.
 *
 * @ingroup solver_evaluators
 */
std::pair<solvers::Binding<
              internal::SlidingFrictionComplementarityNonlinearConstraint>,
          solvers::Binding<StaticFrictionConeConstraint>>
AddSlidingFrictionComplementarityExplicitContactConstraint(
    const ContactWrenchEvaluator* contact_wrench_evaluator,
    double complementarity_tolerance,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& v_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& lambda_vars,
    solvers::MathematicalProgram* prog);

/**
 * For a pair of geometries in implicit contact (they may or may not be in
 * contact, adds the sliding friction complementarity constraint explained in
 * @ref sliding_friction_complementarity_constraint. The input arguments are the
 * same as those in
 * AddSlidingFrictionComplementarityExplicitContactConstraint(). The difference
 * is that the returned argument includes the nonlinear complementarity binding
 * 0 ≤ φ(q) ⊥ fₙ≥ 0, which imposes the constraint for implicit contact.
 *
 * @ingroup solver_evaluators
 */
std::pair<solvers::Binding<
              internal::SlidingFrictionComplementarityNonlinearConstraint>,
          solvers::Binding<
              internal::StaticFrictionConeComplementarityNonlinearConstraint>>
AddSlidingFrictionComplementarityImplicitContactConstraint(
    const ContactWrenchEvaluator* contact_wrench_evaluator,
    double complementarity_tolerance,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& v_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& lambda_vars,
    solvers::MathematicalProgram* prog);
}  // namespace multibody
}  // namespace drake
