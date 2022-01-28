#include "drake/solvers/augmented_lagrangian.h"

#include <fmt/format.h>

#include "drake/common/default_scalars.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

// This function psi is defined as equation 17.55 of Numerical Optimization by
// Jorge Nocedal and Stephen Wright, Edition 1, 1999. (Note this equation is not
// presented in Edition 2). Mathematically it equals psi(c, λ, μ) = -λc +
// 1/(2μ)c² if c - λμ <= 0 otherwise psi(c, λ, μ) = -0.5*μλ² The meaning of this
// function psi(c, λ, μ) is psi(c,λ, μ)= −λ(c−s) + 1/(2μ)(c−s)² where s = max(c
// − μλ, 0)
template <typename T>
T psi(const T& c, double lambda_val, double mu) {
  if (ExtractDoubleOrThrow(c - lambda_val * mu) < 0) {
    return -lambda_val * c + 1 / (2 * mu) * c * c;
  } else {
    return T(-0.5 * mu * lambda_val * lambda_val);
  }
}

NonsmoothAugmentedLagrangian::NonsmoothAugmentedLagrangian(
    const MathematicalProgram* prog, bool include_x_bounds)
    : prog_{prog}, include_x_bounds_{include_x_bounds} {
  lagrangian_size_ = 0;
  for (const auto& constraint : prog_->GetAllConstraints()) {
    if (!dynamic_cast<BoundingBoxConstraint*>(constraint.evaluator().get())) {
      const auto& lb = constraint.evaluator()->lower_bound();
      const auto& ub = constraint.evaluator()->upper_bound();
      for (int i = 0; i < constraint.evaluator()->num_constraints(); ++i) {
        if (lb(i) == ub(i)) {
          lagrangian_size_++;
        } else {
          if (!std::isinf(lb(i))) {
            lagrangian_size_++;
          }
          if (!std::isinf(ub(i))) {
            lagrangian_size_++;
          }
        }
      }
    }
  }
  AggregateBoundingBoxConstraints(*prog_, &x_lo_, &x_up_);
  if (include_x_bounds) {
    for (int i = 0; i < prog_->num_vars(); ++i) {
      if (x_lo_(i) == x_up_(i)) {
        lagrangian_size_++;
      } else {
        if (!std::isinf(x_lo_(i))) {
          lagrangian_size_++;
        }
        if (!std::isinf(x_up_(i))) {
          lagrangian_size_++;
        }
      }
    }
  }
  is_equality_.resize(lagrangian_size_);
  // We loop through all the constraints again instead of combining this loop
  // with the previous loop when we find lagrangian_size_. The reason is that we
  // want to first find lagrangian_size_ and then allocate all the memory of
  // is_equality_.
  int constraint_row = 0;
  for (const auto& constraint : prog_->GetAllConstraints()) {
    if (!dynamic_cast<BoundingBoxConstraint*>(constraint.evaluator().get())) {
      const auto& lb = constraint.evaluator()->lower_bound();
      const auto& ub = constraint.evaluator()->upper_bound();
      for (int i = 0; i < constraint.evaluator()->num_constraints(); ++i) {
        if (lb(i) == ub(i)) {
          is_equality_[constraint_row] = true;
          constraint_row++;
        } else {
          if (!std::isinf(lb(i))) {
            is_equality_[constraint_row] = false;
            constraint_row++;
          }
          if (!std::isinf(ub(i))) {
            is_equality_[constraint_row] = false;
            constraint_row++;
          }
        }
      }
    }
  }
  if (include_x_bounds) {
    for (int i = 0; i < prog_->num_vars(); ++i) {
      if (x_lo_(i) == x_up_(i)) {
        is_equality_[constraint_row] = true;
        constraint_row++;
      } else {
        if (!std::isinf(x_lo_(i))) {
          is_equality_[constraint_row] = false;
          constraint_row++;
        }
        if (!std::isinf(x_up_(i))) {
          is_equality_[constraint_row] = false;
          constraint_row++;
        }
      }
    }
  }
}

template <typename T>
T NonsmoothAugmentedLagrangian::Eval(const Eigen::Ref<const VectorX<T>>& x,
                                     const Eigen::VectorXd& lambda_val,
                                     double mu, VectorX<T>* constraint_residue,
                                     T* cost) const {
  DRAKE_DEMAND(x.rows() == prog_->num_vars());
  DRAKE_DEMAND(lambda_val.rows() == lagrangian_size_);
  DRAKE_DEMAND(mu > 0);
  DRAKE_DEMAND(constraint_residue != nullptr);
  DRAKE_DEMAND(cost != nullptr);
  *cost = T{0};
  constraint_residue->resize(lambda_val.rows());
  for (const auto& cost_binding : prog_->GetAllCosts()) {
    *cost += prog_->EvalBinding(cost_binding, x)(0);
  }
  T al = *cost;
  int lagrangian_count = 0;
  // First evaluate all generic nonlinear constraints
  for (const auto& constraint : prog_->GetAllConstraints()) {
    if (!dynamic_cast<BoundingBoxConstraint*>(constraint.evaluator().get())) {
      const VectorX<T> constraint_val = prog_->EvalBinding(constraint, x);
      // Now check if each row of the constraint is equality or inequality.
      for (int i = 0; i < constraint.evaluator()->num_constraints(); ++i) {
        const double& lb = constraint.evaluator()->lower_bound()(i);
        const double& ub = constraint.evaluator()->upper_bound()(i);
        if ((std::isinf(lb) && lb > 0) || (std::isinf(ub) && ub < 0)) {
          throw std::invalid_argument(fmt::format(
              "constraint lower bound is {}, upper bound is {}", lb, ub));
        }
        if (lb == ub) {
          // We have one Lagrangian multiplier for the equality constraint. Add
          // −λ h(x) + 1/(2μ)h(x)² to the augmented Lagrangian.
          al += -lambda_val(lagrangian_count) * (constraint_val(i) - lb) +
                1. / (2 * mu) * (constraint_val(i) - lb) *
                    (constraint_val(i) - lb);
          (*constraint_residue)(lagrangian_count) = constraint_val(i) - lb;
          lagrangian_count++;
        } else {
          if (!std::isinf(lb)) {
            // The constraint is constraint_val - lb >= 0.
            al += psi(constraint_val(i) - lb, lambda_val(lagrangian_count), mu);
            (*constraint_residue)(lagrangian_count) = constraint_val(i) - lb;
            lagrangian_count++;
          }
          if (!std::isinf(ub)) {
            // The constraint is ub - constraint_val >= 0.
            al += psi(ub - constraint_val(i), lambda_val(lagrangian_count), mu);
            (*constraint_residue)(lagrangian_count) = ub - constraint_val(i);
            lagrangian_count++;
          }
        }
      }
    }
  }
  if (include_x_bounds_) {
    for (int i = 0; i < prog_->num_vars(); ++i) {
      if (x_lo_(i) == x_up_(i)) {
        al += -lambda_val(lagrangian_count) * (x(i) - x_lo_(i)) +
              1. / (2 * mu) * (x(i) - x_lo_(i)) * (x(i) - x_lo_(i));
        (*constraint_residue)(lagrangian_count) = x(i) - x_lo_(i);
        lagrangian_count++;
      } else {
        if (!std::isinf(x_lo_(i))) {
          al += psi(x(i) - x_lo_(i), lambda_val(lagrangian_count), mu);
          (*constraint_residue)(lagrangian_count) = x(i) - x_lo_(i);
          lagrangian_count++;
        }
        if (!std::isinf(x_up_(i))) {
          al += psi(x_up_(i) - x(i), lambda_val(lagrangian_count), mu);
          (*constraint_residue)(lagrangian_count) = x_up_(i) - x(i);
          lagrangian_count++;
        }
      }
    }
  }
  return al;
}

// Explicit instantiation.
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&NonsmoothAugmentedLagrangian::Eval<T>))
}  // namespace solvers
}  // namespace drake
