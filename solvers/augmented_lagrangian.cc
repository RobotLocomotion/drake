#include "drake/solvers/augmented_lagrangian.h"

#include <fmt/format.h>

#include "drake/common/default_scalars.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

namespace {
// This function psi is defined as equation 17.55 of Numerical Optimization by
// Jorge Nocedal and Stephen Wright, Edition 1, 1999. (Note this equation is not
// presented in Edition 2). Mathematically it equals psi(c, λ, μ) = -λc +
// μ/2*c² if c - λ/μ <= 0 otherwise psi(c, λ, μ) = -0.5*λ²/μ The meaning of this
// function psi(c, λ, μ) is psi(c,λ, μ)= −λ(c−s) + μ/2*(c−s)² where s = max(c
// − λ/μ, 0)
// Note that in equation 17.55 of Numerical Optimization, Edition 1, what they
// use for μ is actually 1/μ in our formulation.
template <typename T>
T psi(const T& c, double lambda_val, double mu) {
  if (ExtractDoubleOrThrow(c - lambda_val / mu) < 0) {
    return -lambda_val * c + mu / 2 * c * c;
  } else {
    return T(-0.5 * lambda_val * lambda_val / mu);
  }
}

// Compute the augmented lagrangian for the equality constraint c(x) = 0
// The augmented Lagrangian is −λc+μ/2*c² where c is lhs.
template <typename T>
T al_for_equality(const T& lhs, double lambda_val, double mu) {
  return -lambda_val * lhs + mu / 2 * lhs * lhs;
}

void ParseProgram(const MathematicalProgram* prog, bool include_x_bounds,
                  int* lagrangian_size, std::vector<bool>* is_equality,
                  Eigen::VectorXd* x_lo, Eigen::VectorXd* x_up) {
  *lagrangian_size = 0;
  for (const auto& constraint : prog->GetAllConstraints()) {
    if (!dynamic_cast<BoundingBoxConstraint*>(constraint.evaluator().get())) {
      const auto& lb = constraint.evaluator()->lower_bound();
      const auto& ub = constraint.evaluator()->upper_bound();
      for (int i = 0; i < constraint.evaluator()->num_constraints(); ++i) {
        if (lb(i) == ub(i)) {
          (*lagrangian_size)++;
        } else {
          if (!std::isinf(lb(i))) {
            (*lagrangian_size)++;
          }
          if (!std::isinf(ub(i))) {
            (*lagrangian_size)++;
          }
        }
      }
    }
  }
  AggregateBoundingBoxConstraints(*prog, x_lo, x_up);
  if (include_x_bounds) {
    for (int i = 0; i < prog->num_vars(); ++i) {
      if ((*x_lo)(i) == (*x_up)(i)) {
        (*lagrangian_size)++;
      } else {
        if (!std::isinf((*x_lo)(i))) {
          (*lagrangian_size)++;
        }
        if (!std::isinf((*x_up)(i))) {
          (*lagrangian_size)++;
        }
      }
    }
  }
  is_equality->resize(*lagrangian_size);
  // We loop through all the constraints again instead of combining this loop
  // with the previous loop when we find lagrangian_size_. The reason is that we
  // want to first find lagrangian_size_ and then allocate all the memory of
  // is_equality_.
  int constraint_row = 0;
  for (const auto& constraint : prog->GetAllConstraints()) {
    if (!dynamic_cast<BoundingBoxConstraint*>(constraint.evaluator().get())) {
      const auto& lb = constraint.evaluator()->lower_bound();
      const auto& ub = constraint.evaluator()->upper_bound();
      for (int i = 0; i < constraint.evaluator()->num_constraints(); ++i) {
        if (lb(i) == ub(i)) {
          (*is_equality)[constraint_row] = true;
          constraint_row++;
        } else {
          if (!std::isinf(lb(i))) {
            (*is_equality)[constraint_row] = false;
            constraint_row++;
          }
          if (!std::isinf(ub(i))) {
            (*is_equality)[constraint_row] = false;
            constraint_row++;
          }
        }
      }
    }
  }
  if (include_x_bounds) {
    for (int i = 0; i < prog->num_vars(); ++i) {
      if ((*x_lo)(i) == (*x_up)(i)) {
        (*is_equality)[constraint_row] = true;
        constraint_row++;
      } else {
        if (!std::isinf((*x_lo)(i))) {
          (*is_equality)[constraint_row] = false;
          constraint_row++;
        }
        if (!std::isinf((*x_up)(i))) {
          (*is_equality)[constraint_row] = false;
          constraint_row++;
        }
      }
    }
  }
}
}  // namespace

AugmentedLagrangianNonsmooth::AugmentedLagrangianNonsmooth(
    const MathematicalProgram* prog, bool include_x_bounds)
    : prog_{prog}, include_x_bounds_{include_x_bounds} {
  ParseProgram(prog_, include_x_bounds, &lagrangian_size_, &is_equality_,
               &x_lo_, &x_up_);
}

namespace {
template <typename AL, typename T>
T EvalAugmentedLagrangian(const AL& al, const Eigen::Ref<const VectorX<T>>& x,
                          const Eigen::Ref<const VectorX<T>>& s,
                          const Eigen::VectorXd& lambda_val, double mu,
                          VectorX<T>* constraint_residue, T* cost) {
  DRAKE_DEMAND(x.rows() == al.prog().num_vars());
  if constexpr (std::is_same_v<AL, AugmentedLagrangianSmooth>) {
    DRAKE_DEMAND(al.s_size() == s.rows());
  }
  DRAKE_DEMAND(lambda_val.rows() == al.lagrangian_size());
  DRAKE_DEMAND(mu > 0);
  DRAKE_DEMAND(constraint_residue != nullptr);
  DRAKE_DEMAND(cost != nullptr);
  *cost = T{0};
  constraint_residue->resize(lambda_val.rows());
  for (const auto& cost_binding : al.prog().GetAllCosts()) {
    *cost += al.prog().EvalBinding(cost_binding, x)(0);
  }
  T al_val = *cost;
  int lagrangian_count = 0;
  int s_count = 0;
  // First evaluate all generic nonlinear constraints
  for (const auto& constraint : al.prog().GetAllConstraints()) {
    if (!dynamic_cast<BoundingBoxConstraint*>(constraint.evaluator().get())) {
      const VectorX<T> constraint_val = al.prog().EvalBinding(constraint, x);
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
          // −λ*h(x) + μ/2*h(x)² to the augmented Lagrangian.
          al_val += al_for_equality(constraint_val(i) - lb,
                                    lambda_val(lagrangian_count), mu);
          (*constraint_residue)(lagrangian_count) = constraint_val(i) - lb;
          lagrangian_count++;
        } else {
          if (!std::isinf(lb)) {
            // The constraint is constraint_val - lb >= 0.
            if constexpr (std::is_same_v<AL, AugmentedLagrangianNonsmooth>) {
              al_val +=
                  psi(constraint_val(i) - lb, lambda_val(lagrangian_count), mu);
              (*constraint_residue)(lagrangian_count) = constraint_val(i) - lb;
            } else {
              al_val += al_for_equality(constraint_val(i) - s(s_count) - lb,
                                        lambda_val(lagrangian_count), mu);
              (*constraint_residue)(lagrangian_count) =
                  constraint_val(i) - s(s_count) - lb;
              s_count++;
            }
            lagrangian_count++;
          }
          if (!std::isinf(ub)) {
            // The constraint is ub - constraint_val >= 0.
            if constexpr (std::is_same_v<AL, AugmentedLagrangianNonsmooth>) {
              al_val +=
                  psi(ub - constraint_val(i), lambda_val(lagrangian_count), mu);
              (*constraint_residue)(lagrangian_count) = ub - constraint_val(i);
            } else {
              al_val += al_for_equality(ub - constraint_val(i) - s(s_count),
                                        lambda_val(lagrangian_count), mu);
              (*constraint_residue)(lagrangian_count) =
                  ub - constraint_val(i) - s(s_count);
              s_count++;
            }
            lagrangian_count++;
          }
        }
      }
    }
  }
  if (al.include_x_bounds()) {
    for (int i = 0; i < al.prog().num_vars(); ++i) {
      if (al.x_lo()(i) == al.x_up()(i)) {
        al_val += al_for_equality(x(i) - al.x_lo()(i),
                                  lambda_val(lagrangian_count), mu);
        (*constraint_residue)(lagrangian_count) = x(i) - al.x_lo()(i);
        lagrangian_count++;
      } else {
        if (!std::isinf(al.x_lo()(i))) {
          if constexpr (std::is_same_v<AL, AugmentedLagrangianNonsmooth>) {
            al_val +=
                psi(x(i) - al.x_lo()(i), lambda_val(lagrangian_count), mu);
            (*constraint_residue)(lagrangian_count) = x(i) - al.x_lo()(i);
          } else {
            al_val += al_for_equality(x(i) - al.x_lo()(i) - s(s_count),
                                      lambda_val(lagrangian_count), mu);
            (*constraint_residue)(lagrangian_count) =
                x(i) - al.x_lo()(i) - s(s_count);
          }
          s_count++;
          lagrangian_count++;
        }
        if (!std::isinf(al.x_up()(i))) {
          if constexpr (std::is_same_v<AL, AugmentedLagrangianNonsmooth>) {
            al_val +=
                psi(al.x_up()(i) - x(i), lambda_val(lagrangian_count), mu);
            (*constraint_residue)(lagrangian_count) = al.x_up()(i) - x(i);
          } else {
            al_val += al_for_equality(al.x_up()(i) - x(i) - s(s_count),
                                      lambda_val(lagrangian_count), mu);
            (*constraint_residue)(lagrangian_count) =
                al.x_up()(i) - x(i) - s(s_count);
            s_count++;
          }
          lagrangian_count++;
        }
      }
    }
  }
  return al_val;
}
}  // namespace

template <typename T>
T AugmentedLagrangianNonsmooth::Eval(const Eigen::Ref<const VectorX<T>>& x,
                                     const Eigen::VectorXd& lambda_val,
                                     double mu, VectorX<T>* constraint_residue,
                                     T* cost) const {
  Eigen::Matrix<T, 0, 1> s_dummy;
  return EvalAugmentedLagrangian<AugmentedLagrangianNonsmooth, T>(
      *this, x, s_dummy, lambda_val, mu, constraint_residue, cost);
}

AugmentedLagrangianSmooth::AugmentedLagrangianSmooth(
    const MathematicalProgram* prog, bool include_x_bounds)
    : prog_{prog}, include_x_bounds_{include_x_bounds} {
  ParseProgram(prog_, include_x_bounds, &lagrangian_size_, &is_equality_,
               &x_lo_, &x_up_);
  // For each inequality constraint, we need one slack variable s.
  s_size_ = 0;
  for (auto flag : is_equality_) {
    if (!flag) {
      s_size_++;
    }
  }
}

template <typename T>
T AugmentedLagrangianSmooth::Eval(const Eigen::Ref<const VectorX<T>>& x,
                                  const Eigen::Ref<const VectorX<T>>& s,
                                  const Eigen::VectorXd& lambda_val, double mu,
                                  VectorX<T>* constraint_residue,
                                  T* cost) const {
  return EvalAugmentedLagrangian<AugmentedLagrangianSmooth, T>(
      *this, x, s, lambda_val, mu, constraint_residue, cost);
}

// Explicit instantiation.
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&AugmentedLagrangianNonsmooth::Eval<T>,
     &AugmentedLagrangianSmooth::Eval<T>));
}  // namespace solvers
}  // namespace drake
