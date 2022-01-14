#include "drake/solvers/augmented_lagrangian.h"

#include <fmt/format.h>

#include "drake/common/default_scalars.h"
#include "drake/solvers/aggregate_costs_constraints.h"

namespace drake {
namespace solvers {
namespace internal {

// This function psi is defined as equation 17.55 of Numerical Optimization by
// Jorge Nocedal and Stephen Wright.
// Mathematically it equals to
// psi(c, λ, μ) = -λc + 1/(2μ)c² if c - λμ <= 0
// otherwise
// psi(c, λ, μ) = -0.5*μλ²
// The meaning of this function psi(c, λ, μ) is
// psi(c,λ, μ)= −λ(c−s) + 1/(2μ)(c−s)² where s = max(c − μλ, 0)
template <typename T>
T psi(const T& c, double lambda, double mu) {
  if (ExtractDoubleOrThrow(c - lambda * mu) < 0) {
    return -lambda * c + 1 / (2 * mu) * c * c;
  } else {
    return T(-0.5 * mu * lambda * lambda);
  }
}

template <typename T>
T EvalAugmentedLagrangian(const MathematicalProgram& prog,
                          const Eigen::Ref<const VectorX<T>>& x,
                          const Eigen::VectorXd& lambda, double mu,
                          bool include_x_bounds) {
  DRAKE_DEMAND(x.rows() == prog.num_vars());
  DRAKE_DEMAND(mu > 0);
  T al{0};
  for (const auto& cost : prog.GetAllCosts()) {
    al += prog.EvalBinding(cost, x)(0);
  }
  int lagrangian_count = 0;
  // First evaluate all generic nonlinear constraints
  for (const auto& constraint : prog.GetAllConstraints()) {
    if (!dynamic_cast<BoundingBoxConstraint*>(constraint.evaluator().get())) {
      const VectorX<T> constraint_val = prog.EvalBinding(constraint, x);
      // Now check if each row of the constraint is equality or inequality.
      for (int i = 0; i < constraint.evaluator()->num_constraints(); ++i) {
        const double& lb = constraint.evaluator()->lower_bound()(i);
        const double& ub = constraint.evaluator()->upper_bound()(i);
        if ((std::isinf(lb) && lb > 0) || (std::isinf(ub) && ub < 0)) {
          throw std::invalid_argument(fmt::format(
              "constraint lower bound is {}, upper bound is {}", lb, ub));
        }
        if (lb == ub) {
          // We have one lagrangian multiplier for the equality constraint. Add
          // −λ h(x) + 1/(2μ)h(x)² to the augmented lagrangian
          al += -lambda(lagrangian_count) * (constraint_val(i) - lb) +
                1. / (2 * mu) * (constraint_val(i) - lb) *
                    (constraint_val(i) - lb);
          lagrangian_count++;
        } else {
          if (!std::isinf(lb)) {
            // The constraint is constraint_val - lb >= 0
            al += psi(constraint_val(i) - lb, lambda(lagrangian_count), mu);
            lagrangian_count++;
          }
          if (!std::isinf(ub)) {
            // The constraint is ub - constraint_val >= 0
            al += psi(ub - constraint_val(i), lambda(lagrangian_count), mu);
            lagrangian_count++;
          }
        }
      }
    }
  }
  if (include_x_bounds) {
    Eigen::VectorXd x_lo, x_up;
    AggregateBoundingBoxConstraints(prog, &x_lo, &x_up);
    for (int i = 0; i < prog.num_vars(); ++i) {
      if (x_lo(i) == x_up(i)) {
        al += -lambda(lagrangian_count) * (x(i) - x_lo(i)) +
              1. / (2 * mu) * (x(i) - x_lo(i)) * (x(i) - x_lo(i));
        lagrangian_count++;
      } else {
        if (!std::isinf(x_lo(i))) {
          al += psi(x(i) - x_lo(i), lambda(lagrangian_count), mu);
          lagrangian_count++;
        }
        if (!std::isinf(x_up(i))) {
          al += psi(x_up(i) - x(i), lambda(lagrangian_count), mu);
          lagrangian_count++;
        }
      }
    }
  }
  DRAKE_DEMAND(lagrangian_count == lambda.rows());
  return al;
}

// Explicit instantiation.
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&EvalAugmentedLagrangian<T>))
}  // namespace internal
}  // namespace solvers
}  // namespace drake
