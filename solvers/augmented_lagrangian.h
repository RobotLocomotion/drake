#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
/**
 * Compute the augmented Lagrangian (AL) of a given mathematical program
 *
 *     min f(x)
 *     s.t h(x) = 0
 *         l <= g(x) <= u
 *         x_lo <= x <= x_up
 *
 * We first turn it into an equality constrained program with non-negative slack
 * variable s as follows
 *
 *     min f(x)
 *     s.t h(x) = 0
 *         c(x) - s = 0
 *         s >= 0
 *
 * Depending on the option include_x_bounds, the constraint h(x)=0, c(x)>=0 may
 * or may not include the bounding box constraint x_lo <= x <= x_up.
 *
 * the (non-smooth) augmented Lagrangian is defined as
 *
 *     L(x, λ, μ) = f(x) − λ₁ᵀh(x) + μ/2 h(x)ᵀh(x)
 *                  - λ₂ᵀ(c(x)-s) + μ/2 (c(x)-s)ᵀ(c(x)-s)
 *
 * where s = max(c(x) - λ₂/μ, 0).
 *
 * For more details, refer to section 17.4 of Numerical Optimization by Jorge
 * Nocedal and Stephen Wright, Edition 1, 1999 (This formulation isn't presented
 * in Edition 2, but to stay consistent with Edition 2, we use μ/2 as the
 * coefficient of the quadratic penalty term instead of 1/(2μ) in Edition 1).
 * Note that the augmented Lagrangian L(x, λ, μ) is NOT a smooth function of x,
 * since s = max(c(x) - λ₂/μ, 0) is non-smooth at c(x) - λ₂/μ = 0.
 */
class AugmentedLagrangianNonsmooth {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AugmentedLagrangianNonsmooth);

  /**
   * @param prog The mathematical program we will evaluate.
   * @param include_x_bounds Whether the Lagrangian and the penalty for the
   * bounds x_lo <= x <= x_up are included in the augmented Lagrangian L(x, λ,
   * μ) or not.
   */
  AugmentedLagrangianNonsmooth(const MathematicalProgram* prog,
                               bool include_x_bounds);

  /**
   * @param x The value of all the decision variables.
   * @param lambda_val The estimated Lagrangian multipliers. The order of the
   * Lagrangian multiplier is as this: We first call to evaluate all
   * constraints. Then for each row of the constraint, if it is an equality
   * constraint, we append one single Lagrangian multiplier. Otherwise we
   * append the Lagrangian multiplier for the lower and upper bounds (where the
   * lower comes before the upper), if the corresponding bound is not ±∞. The
   * order of evaluating all the constraints is the same as
   * prog.GetAllConstraints() except for prog.bounding_box_constraints(). If
   * include_x_bounds=true, then we aggregate all the bounding_box_constraints()
   * and evaluate them at the end of all constraints.
   * @param mu μ in the documentation above. The constant for penalty term
   * weight. This should be a strictly positive number.
   * @param[out] constraint_residue The value of the all the constraints. For an
   * equality constraint c(x)=0 or the inequality constraint c(x)>= 0, the
   * residue is c(x). Depending on include_x_bounds, `constraint_residue` may or
   * may not contain the residue for bounding box constraints x_lo <= x <= x_up
   * at the end.
   * @param[out] cost The value of the cost function f(x).
   * @return The evaluated Augmented Lagrangian (AL) L(x, λ, μ).
   */
  template <typename T>
  T Eval(const Eigen::Ref<const VectorX<T>>& x,
         const Eigen::VectorXd& lambda_val, double mu,
         VectorX<T>* constraint_residue, T* cost) const;

  /**
   * @return The mathematical program for which the augmented Lagrangian is
   * computed.
   */
  [[nodiscard]] const MathematicalProgram& prog() const { return *prog_; }

  /**
   * @return Whether the bounding box constraint x_lo <= x <=
   * x_up is included in the augmented Lagrangian L(x, λ, μ).
   */
  [[nodiscard]] bool include_x_bounds() const { return include_x_bounds_; }

  /**
   * @return The size of the Lagrangian multiplier λ.
   */
  [[nodiscard]] int lagrangian_size() const { return lagrangian_size_; }

  /**
   * @return Whether each constraint is equality or not. The order of the
   * constraint is explained in the class documentation.
   */
  [[nodiscard]] const std::vector<bool>& is_equality() const {
    return is_equality_;
  }

  /** @return all the lower bounds of x. */
  [[nodiscard]] const Eigen::VectorXd& x_lo() const { return x_lo_; }

  /** @return all the upper bounds of x. */
  [[nodiscard]] const Eigen::VectorXd& x_up() const { return x_up_; }

 private:
  const MathematicalProgram* prog_;
  bool include_x_bounds_;
  int lagrangian_size_;
  std::vector<bool> is_equality_;
  Eigen::VectorXd x_lo_;
  Eigen::VectorXd x_up_;
};

/**
 * Compute the augmented Lagrangian (AL) of a given mathematical program
 *
 *     min f(x)
 *     s.t h(x) = 0
 *         l <= g(x) <= u
 *         x_lo <= x <= x_up
 *
 * We first turn it into an equality constrained program with non-negative slack
 * variable s as follows
 *
 *     min f(x)
 *     s.t h(x) = 0
 *         c(x) - s = 0
 *         s >= 0
 *
 * We regard this as an optimization problem on variable (x, s), with equality
 * constraints h(x) = 0, c(x)-s = 0, and the bound constraint s >= 0.
 *
 * Depending on the option include_x_bounds, the constraint h(x)=0, c(x)>=0 may
 * or may not include the bounding box constraint x_lo <= x <= x_up.
 *
 * The (smooth) augmented Lagrangian is defined as
 *
 *     L(x, s, λ, μ) = f(x) − λ₁ᵀh(x) + μ/2 h(x)ᵀh(x)
 *                  - λ₂ᵀ(c(x)-s) + μ/2 (c(x)-s)ᵀ(c(x)-s)
 *
 * For more details, refer to section 17.4 of Numerical Optimization by Jorge
 * Nocedal and Stephen Wright, Edition 2, 2006.
 * Note that the augmented Lagrangian L(x, s, λ, μ) is a smooth function of (x,
 * s),
 *
 * This is the implementation used in LANCELOT. To solve the nonlinear
 * optimization through this Augmented Lagrangian, the nonlinear solve should be
 * able to handle bounding box constraints on the decision variables.
 */
class AugmentedLagrangianSmooth {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AugmentedLagrangianSmooth);

  /**
   * @param prog The mathematical program we will evaluate.
   * @param include_x_bounds Whether the Lagrangian and the penalty for the
   * bounds x_lo <= x <= x_up are included in the augmented Lagrangian L(x, s,
   * λ, μ) or not.
   */
  AugmentedLagrangianSmooth(const MathematicalProgram* prog,
                            bool include_x_bounds);

  /**
   * @param x The value of all the decision variables in prog().
   * @param s The value of all slack variables s.
   * @param lambda_val The estimated Lagrangian multipliers. The order of the
   * Lagrangian multiplier is as follows: We first call to evaluate all
   * constraints. Then for each row of the constraint, if it is an equality
   * constraint, we append one single Lagrangian multiplier. Otherwise we
   * append the Lagrangian multiplier for the lower and upper bounds (where the
   * lower comes before the upper), if the corresponding bound is not ±∞. The
   * order of evaluating all the constraints is the same as
   * prog.GetAllConstraints() except for prog.bounding_box_constraints(). If
   * include_x_bounds=true, then we aggregate all the bounding_box_constraints()
   * and evaluate them at the end of all constraints.
   * @param mu μ in the documentation above. The constant for penalty term
   * weight. This should be a strictly positive number.
   * @param[out] constraint_residue The value of the all the constraints. For an
   * equality constraint c(x)=0, the residue is c(x); for an inequality
   * constraint c(x)>=0, the residue is c(x)-s where s is the corresponding
   * slack variable. Depending on include_x_bounds, `constraint_residue` may or
   * may not contain the residue for bounding box constraints x_lo <= x <= x_up
   * at the end.
   * @param[out] cost The value of the cost function f(x).
   * @return The evaluated Augmented Lagrangian (AL) L(x, s, λ, μ).
   * @note This Eval function differs from AugmentedLagrangianNonsmooth::Eval()
   * function as `s` is an input argument.
   */
  template <typename T>
  T Eval(const Eigen::Ref<const VectorX<T>>& x,
         const Eigen::Ref<const VectorX<T>>& s,
         const Eigen::VectorXd& lambda_val, double mu,
         VectorX<T>* constraint_residue, T* cost) const;

  /**
   * @return The mathematical program for which the augmented Lagrangian is
   * computed.
   */
  [[nodiscard]] const MathematicalProgram& prog() const { return *prog_; }

  /**
   * @return Whether the bounding box constraint x_lo <= x <=
   * x_up is included in the augmented Lagrangian L(x, λ, μ).
   */
  [[nodiscard]] bool include_x_bounds() const { return include_x_bounds_; }

  /**
   * @return The size of the Lagrangian multiplier λ.
   */
  [[nodiscard]] int lagrangian_size() const { return lagrangian_size_; }

  /**
   * @return The size of the slack variable s.
   */
  [[nodiscard]] int s_size() const { return s_size_; }

  /**
   * @return Whether each constraint is equality or not. The order of the
   * constraint is explained in the class documentation.
   */
  [[nodiscard]] const std::vector<bool>& is_equality() const {
    return is_equality_;
  }

  /** @return All the lower bounds of x. */
  [[nodiscard]] const Eigen::VectorXd& x_lo() const { return x_lo_; }

  /** @return All the upper bounds of x. */
  [[nodiscard]] const Eigen::VectorXd& x_up() const { return x_up_; }

 private:
  const MathematicalProgram* prog_;
  bool include_x_bounds_;
  int lagrangian_size_;
  int s_size_{0};
  std::vector<bool> is_equality_;
  Eigen::VectorXd x_lo_;
  Eigen::VectorXd x_up_;
};
}  // namespace solvers
}  // namespace drake
