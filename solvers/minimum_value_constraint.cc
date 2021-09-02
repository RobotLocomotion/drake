#include "drake/solvers/minimum_value_constraint.h"

#include <limits>
#include <vector>

#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace solvers {

namespace {
/** Computes log(∑exp(xᵢ)). Exploits the shift invariance of log-sum-exp to
avoid overflow. */
template <typename T>
T LogSumExp(const std::vector<T>& x) {
  DRAKE_ASSERT(x.size() > 0);
  using std::exp;
  using std::log;
  const T x_max = *std::max_element(x.begin(), x.end());
  T sum_exp{0.0};
  for (const T& xi : x) {
    sum_exp += exp(xi - x_max);
  }
  return x_max + log(sum_exp);
}

/** Computes a smooth approximation of max(x). */
template <typename T>
T SmoothMax(const std::vector<T>& x) {
  // We compute the smooth max of x as smoothmax(x) = log(∑ᵢ exp(αxᵢ)) / α.
  // This smooth max approaches max(x) as α increases. We choose α = 100, as
  // that gives a qualitatively good fit for xᵢ ∈ [0, 1], which is the range of
  // potential penalty values when the MinimumValueConstraint is feasible.
  DRAKE_ASSERT(x.size() > 0);
  const double alpha{100};
  std::vector<T> x_scaled{x};
  for (T& xi_scaled : x_scaled) {
    xi_scaled *= alpha;
  }
  return LogSumExp(x_scaled) / alpha;
}

template <typename T>
T ScaleValue(T value, double minimum_value, double influence_value) {
  return (value - influence_value) / (influence_value - minimum_value);
}

void InitializeY(const Eigen::Ref<const Eigen::VectorXd>&, Eigen::VectorXd* y,
                 double y_value) {
  (*y)(0) = y_value;
}

void InitializeY(const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y,
                 double y_value) {
  (*y) = math::InitializeAutoDiff(
      Vector1d(y_value), Eigen::RowVectorXd::Zero(x(0).derivatives().size()));
}

void Penalty(const double& value, double minimum_value, double influence_value,
             MinimumValuePenaltyFunction penalty_function, double* y) {
  double penalty;
  const double x = ScaleValue(value, minimum_value, influence_value);
  penalty_function(x, &penalty, nullptr);
  *y = penalty;
}

void Penalty(const AutoDiffXd& value, double minimum_value,
             double influence_value,
             MinimumValuePenaltyFunction penalty_function, AutoDiffXd* y) {
  const AutoDiffXd scaled_value_autodiff =
      ScaleValue(value, minimum_value, influence_value);
  double penalty, dpenalty_dscaled_value;
  penalty_function(scaled_value_autodiff.value(), &penalty,
                   &dpenalty_dscaled_value);

  const Vector1<AutoDiffXd> penalty_autodiff = math::InitializeAutoDiff(
      Vector1d(penalty),
      dpenalty_dscaled_value *
          math::ExtractGradient(Vector1<AutoDiffXd>{scaled_value_autodiff}));
  *y = penalty_autodiff(0);
}

}  // namespace

void ExponentiallySmoothedHingeLoss(double x, double* penalty,
                                    double* dpenalty_dx) {
  if (x >= 0) {
    *penalty = 0;
    if (dpenalty_dx) {
      *dpenalty_dx = 0;
    }
  } else {
    const double exp_one_over_x = std::exp(1.0 / x);
    *penalty = -x * exp_one_over_x;
    if (dpenalty_dx) {
      *dpenalty_dx = -exp_one_over_x + exp_one_over_x / x;
    }
  }
}

void QuadraticallySmoothedHingeLoss(double x, double* penalty,
                                    double* dpenalty_dx) {
  if (x >= 0) {
    *penalty = 0;
    if (dpenalty_dx) {
      *dpenalty_dx = 0;
    }
  } else {
    if (x > -1) {
      *penalty = x * x / 2;
      if (dpenalty_dx) {
        *dpenalty_dx = x;
      }
    } else {
      *penalty = -0.5 - x;
      if (dpenalty_dx) {
        *dpenalty_dx = -1;
      }
    }
  }
}

MinimumValueConstraint::MinimumValueConstraint(
    int num_vars, double minimum_value, double influence_value_offset,
    int max_num_values,
    std::function<AutoDiffVecXd(const Eigen::Ref<const AutoDiffVecXd>&, double)>
        value_function,
    std::function<VectorX<double>(const Eigen::Ref<const VectorX<double>>&,
                                  double)>
        value_function_double)
    : solvers::Constraint(1, num_vars,
                          Vector1d(-std::numeric_limits<double>::infinity()),
                          Vector1d(1)),
      value_function_{value_function},
      value_function_double_{value_function_double},
      minimum_value_{minimum_value},
      influence_value_{minimum_value + influence_value_offset},
      max_num_values_{max_num_values} {
  set_penalty_function(QuadraticallySmoothedHingeLoss);
}

void MinimumValueConstraint::set_penalty_function(
    MinimumValuePenaltyFunction new_penalty_function) {
  penalty_function_ = new_penalty_function;
  double unscaled_penalty_at_minimum_value{};
  penalty_function_(
      ScaleValue(minimum_value_, minimum_value_, influence_value_),
      &unscaled_penalty_at_minimum_value, nullptr);
  penalty_output_scaling_ = 1 / unscaled_penalty_at_minimum_value;
}

template <>
VectorX<double> MinimumValueConstraint::Values(
    const Eigen::Ref<const VectorX<double>>& x) const {
  return value_function_double_
             ? value_function_double_(x, influence_value_)
             : math::ExtractValue(value_function_(math::InitializeAutoDiff(x),
                                                  influence_value_));
}

template <>
AutoDiffVecXd MinimumValueConstraint::Values(
    const Eigen::Ref<const AutoDiffVecXd>& x) const {
  return value_function_(x, influence_value_);
}

template <typename T>
void MinimumValueConstraint::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  y->resize(1);

  // If we know that Values() will return at most zero values, then this
  // is a non-constraint. Return zero in that case.
  if (max_num_values_ == 0) {
    InitializeY(x, y, 0.0);
    return;
  }

  // Initialize y to SmoothMax([0, 0, ..., 0]).
  InitializeY(x, y, SmoothMax(std::vector<double>(max_num_values_, 0.0)));

  VectorX<T> values = Values(x);
  std::vector<T> penalties{};
  const int num_values = static_cast<int>(values.size());
  DRAKE_ASSERT(num_values <= max_num_values_);
  penalties.reserve(max_num_values_);
  for (int i = 0; i < num_values; ++i) {
    const T& value = values(i);
    if (value < influence_value_) {
      penalties.emplace_back();
      Penalty(value, minimum_value_, influence_value_, penalty_function_,
              &penalties.back());
      penalties.back() *= penalty_output_scaling_;
    }
  }
  if (!penalties.empty()) {
    // Pad penalties up to max_num_values_ so that the constraint
    // function is actually smooth.
    penalties.resize(max_num_values_, T{0.0});
    (*y)(0) = SmoothMax(penalties);
  }
}

void MinimumValueConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                    Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void MinimumValueConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                    AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}
}  // namespace solvers
}  // namespace drake
