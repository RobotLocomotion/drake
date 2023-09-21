#include "drake/solvers/minimum_value_constraint.h"

#include <limits>

#include "drake/math/autodiff_gradient.h"
#include "drake/math/soft_min_max.h"

namespace drake {
namespace solvers {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

using ValueFunctionDoubleType =
    std::function<VectorX<double>(const VectorX<double>&, double)>;
using ValueFunctionAutodiffType =
    std::function<AutoDiffVecXd(const AutoDiffVecXd&, double)>;

/** Computes a smooth over approximation of max(x). */
template <typename T>
T SmoothOverMax(const std::vector<T>& x) {
  // We compute the smooth max of x as smoothmax(x) = log(∑ᵢ exp(αxᵢ)) / α.
  // This smooth max approaches max(x) as α increases. We choose α = 100, as
  // that gives a qualitatively good fit for xᵢ ∈ [0, 1], which is the range of
  // potential penalty values when the MinimumValueConstraint is feasible.
  return math::SoftOverMax(x, 100 /* alpha */);
}

/** Computes a smooth under approximation of max(x). */
template <typename T>
T SmoothUnderMax(const std::vector<T>& x) {
  // This SoftUnderMax approaches max(x) as α increases. We choose α = 100, as
  // that gives a qualitatively good fit for xᵢ around 1.
  return math::SoftUnderMax(x, 100 /* alpha */);
}

template <typename T>
T ScaleValue(T value, double minimum_value, double influence_value) {
  return (value - influence_value) / (influence_value - minimum_value);
}

void InitializeY(const Eigen::Ref<const Eigen::VectorXd>&, Eigen::VectorXd* y,
                 int y_index, double y_value) {
  (*y)(y_index) = y_value;
}

void InitializeY(const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y,
                 int y_index, double y_value) {
  (*y)(y_index).value() = y_value;
  (*y)(y_index).derivatives() =
      Eigen::RowVectorXd::Zero(x(0).derivatives().size());
}

void Penalty(const double& value, double minimum_value, double influence_value,
             const MinimumValuePenaltyFunction& penalty_function, double* y) {
  double penalty;
  const double x = ScaleValue(value, minimum_value, influence_value);
  penalty_function(x, &penalty, nullptr);
  *y = penalty;
}

void Penalty(const AutoDiffXd& value, double minimum_value,
             double influence_value,
             const MinimumValuePenaltyFunction& penalty_function,
             AutoDiffXd* y) {
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

void set_penalty_function_impl(MinimumValuePenaltyFunction new_penalty_function,
                               MinimumValuePenaltyFunction* penalty_function,
                               double* penalty_output_scaling) {
  *penalty_function = std::move(new_penalty_function);
  double unscaled_penalty_at_minimum_value{};
  (*penalty_function)(-1, &unscaled_penalty_at_minimum_value, nullptr);
  *penalty_output_scaling = 1 / unscaled_penalty_at_minimum_value;
}

VectorX<double> Values(const Eigen::Ref<const VectorX<double>>& x,
                       double influence_value,
                       const ValueFunctionDoubleType& value_function_double,
                       const ValueFunctionAutodiffType& value_function_ad) {
  return value_function_double ? value_function_double(x, influence_value)
                               : math::ExtractValue(value_function_ad(
                                     x.cast<AutoDiffXd>(), influence_value));
}

AutoDiffVecXd Values(const Eigen::Ref<const AutoDiffVecXd>& x,
                     double influence_value,
                     const ValueFunctionDoubleType& value_function_double,
                     const ValueFunctionAutodiffType& value_function_ad) {
  unused(value_function_double);
  return value_function_ad(x, influence_value);
}

// Given v, evaluate
// smooth_fun( φ((vᵢ - v_influence)/(v_influence - bound_value)) / φ(-1) )
template <typename T>
void EvalPenalty(double bound_value, const VectorX<T>& values,
                 int max_num_values, double influence_value,
                 MinimumValuePenaltyFunction penalty_function,
                 double penalty_output_scaling,
                 const std::function<T(const std::vector<T>&)>& smooth_func,
                 T* y) {
  const int num_values = values.size();
  std::vector<T> penalties;
  penalties.reserve(max_num_values);
  for (int i = 0; i < num_values; ++i) {
    const T& value = values(i);
    if (value < influence_value) {
      penalties.emplace_back();
      Penalty(value, bound_value, influence_value, penalty_function,
              &(penalties.back()));
      penalties.back() *= penalty_output_scaling;
    }
  }

  if (!penalties.empty()) {
    // Pad penalties up to max_num_values_ so that the constraint
    // function is actually smooth.
    penalties.resize(max_num_values, T{0.0});
    (*y) = smooth_func(penalties);
  }
}

template <typename T>
void EvalGeneric(
    double bound, int max_num_values, double y_for_empty_value,
    double influence_value,
    const ValueFunctionDoubleType& value_function_double,
    const ValueFunctionAutodiffType& value_function_ad,
    const std::function<double(const std::vector<double>&)>& smooth_func_double,
    const std::function<T(const std::vector<T>&)>& smooth_func,
    const MinimumValuePenaltyFunction& penalty_function,
    double penalty_output_scaling, const Eigen::Ref<const VectorX<T>>& x,
    VectorX<T>* y) {
  y->resize(1);
  // If we know that Values() will return at most zero values, then this
  // is a non-constraint. Return zero for the lower bound constraint
  if (max_num_values == 0) {
    InitializeY(x, y, 0, y_for_empty_value);
    return;
  }
  // Initialize y to SmoothOverMax([0,..., 0])
  // We choose 0 because that is φ((v - v_influence)/(v_influence -
  // bound_value)) / φ(-1) when v = v_influence.
  InitializeY(x, y, 0,
              smooth_func_double(std::vector<double>(max_num_values, 0.0)));

  VectorX<T> values =
      Values(x, influence_value, value_function_double, value_function_ad);
  const int num_values = static_cast<int>(values.size());
  DRAKE_ASSERT(num_values <= max_num_values);
  EvalPenalty<T>(bound, values, max_num_values, influence_value,
                 penalty_function, penalty_output_scaling, smooth_func,
                 y->data());
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
    : MinimumValueConstraint(num_vars, minimum_value, kInf,
                             minimum_value + influence_value_offset,
                             max_num_values, std::move(value_function),
                             std::move(value_function_double)) {}

namespace {
int NumConstraints(double minimum_value_lower, double minimum_value_upper) {
  if (std::isfinite(minimum_value_lower) ||
      std::isfinite(minimum_value_upper)) {
    return static_cast<int>(std::isfinite(minimum_value_lower)) +
           static_cast<int>(std::isfinite(minimum_value_upper));
  } else {
    throw std::runtime_error(fmt::format(
        "MinimumValueConstraint: minimum_value_lower={}, "
        "minimum_value_upper={}. At least one of them should be finite.",
        minimum_value_lower, minimum_value_upper));
  }
}

Eigen::VectorXd LowerBounds(double minimum_value_lower,
                            double minimum_value_upper) {
  // We assume at least one of minimum_value_lower and minimum_value_upper is
  // finite, as checked already in NumConstraints().
  if (std::isfinite(minimum_value_lower) &&
      std::isfinite(minimum_value_upper)) {
    return Eigen::Vector2d(-kInf, 1);
  } else if (std::isfinite(minimum_value_lower)) {
    return Vector1d(-kInf);
  } else if (std::isfinite(minimum_value_upper)) {
    return Vector1d(1);
  }
  DRAKE_UNREACHABLE();
}

Eigen::VectorXd UpperBounds(double minimum_value_lower,
                            double minimum_value_upper) {
  // We assume at least one of minimum_value_lower and minimum_value_upper is
  // finite, as checked already in NumConstraints().
  if (std::isfinite(minimum_value_lower) &&
      std::isfinite(minimum_value_upper)) {
    return Eigen::Vector2d(1, kInf);
  } else if (std::isfinite(minimum_value_lower)) {
    return Vector1d(1);
  } else if (std::isfinite(minimum_value_upper)) {
    return Vector1d(kInf);
  }
  DRAKE_UNREACHABLE();
}
}  // namespace

MinimumValueConstraint::MinimumValueConstraint(
    int num_vars, double minimum_value_lower, double minimum_value_upper,
    double influence_value, int max_num_values,
    std::function<AutoDiffVecXd(const Eigen::Ref<const AutoDiffVecXd>&, double)>
        value_function,
    std::function<VectorX<double>(const Eigen::Ref<const VectorX<double>>&,
                                  double)>
        value_function_double)
    : Constraint(NumConstraints(minimum_value_lower, minimum_value_upper),
                 num_vars,
                 LowerBounds(minimum_value_lower, minimum_value_upper),
                 UpperBounds(minimum_value_lower, minimum_value_upper)),
      value_function_{std::move(value_function)},
      value_function_double_{std::move(value_function_double)},
      minimum_value_lower_{minimum_value_lower},
      minimum_value_upper_{minimum_value_upper},
      influence_value_{influence_value},
      max_num_values_{max_num_values} {
  DRAKE_DEMAND(!std::isnan(minimum_value_lower_));
  DRAKE_DEMAND(!std::isnan(minimum_value_upper_));
  DRAKE_DEMAND(influence_value_ > minimum_value_lower_);
  if (std::isfinite(minimum_value_upper)) {
    DRAKE_DEMAND(influence_value_ > minimum_value_upper_);
  }
  DRAKE_DEMAND(std::isfinite(influence_value_));
  this->set_penalty_function(QuadraticallySmoothedHingeLoss);
}

void MinimumValueConstraint::set_penalty_function(
    MinimumValuePenaltyFunction new_penalty_function) {
  set_penalty_function_impl(new_penalty_function, &penalty_function_,
                            &penalty_output_scaling_);
}

template <typename T>
void MinimumValueConstraint::EvalMaxPenalty(
    double bound_value, const VectorX<T>& values, int y_index,
    const std::function<T(const std::vector<T>&)>& smooth_func,
    VectorX<T>* y) const {
  if (y_index >= 0) {
    EvalPenalty<T>(bound_value, values, max_num_values_, influence_value_,
                   penalty_function_, penalty_output_scaling_, smooth_func,
                   y->data() + y_index);
  }
}

template <typename T>
void MinimumValueConstraint::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  y->resize(num_constraints());

  // Indices for the one or two possible constraints. Infinite bound valuess map
  // to invalid index values (-1).
  // SoftOverMax(φ((vᵢ - v_influence)/(v_influence - vₘᵢₙ_lower)) / φ(-1)) ≤ 1.
  const int y_lower_index = std::isfinite(minimum_value_lower_) ? 0 : -1;
  // Index for the constraint
  // SoftUnderMax( φ((vᵢ - v_influence)/(v_influence - vₘᵢₙ_upper)) / φ(-1) ) ≥
  // 1.
  const int y_upper_index =
      std::isfinite(minimum_value_upper_) ? y_lower_index + 1 : -1;

  // If we know that Values() will return at most zero values, then this
  // is a non-constraint. Return zero for the lower bound constraint, and 2 for
  // the upper bound constraint.
  if (max_num_values_ == 0) {
    if (y_lower_index >= 0) {
      // The upper bound is 1. Set the value to 0 to guarantee that the
      // constraint is satisfied.
      InitializeY(x, y, y_lower_index, 0.0);
    }
    if (y_upper_index >= 0) {
      // The lower bound is 1. Set the value to 2 to guarantee that the
      // constraint is satisfied.
      InitializeY(x, y, y_upper_index, 2.0);
    }
    return;
  }

  // Initialize y(y_lower_index) and y(y_upper_index) to
  // SmoothOverMax([0,..., 0]) and SmoothUnderMax([0, ..., 0]), respectively.
  if (y_lower_index >= 0) {
    InitializeY(x, y, y_lower_index,
                SmoothOverMax(std::vector<double>(max_num_values_, 0.0)));
  }
  if (y_upper_index >= 0) {
    InitializeY(x, y, y_upper_index,
                SmoothUnderMax(std::vector<double>(max_num_values_, 0.0)));
  }

  VectorX<T> values =
      Values(x, influence_value_, value_function_double_, value_function_);
  const int num_values = static_cast<int>(values.size());
  DRAKE_ASSERT(num_values <= max_num_values_);
  this->EvalMaxPenalty<T>(
      minimum_value_lower_, values, y_lower_index,
      [](const std::vector<T>& v) {
        return SmoothOverMax<T>(v);
      },
      y);
  this->EvalMaxPenalty<T>(
      minimum_value_upper_, values, y_upper_index,
      [](const std::vector<T>& v) {
        return SmoothUnderMax<T>(v);
      },
      y);
}

void MinimumValueConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                    Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void MinimumValueConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                    AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}

MinimumValueLowerBoundConstraint::MinimumValueLowerBoundConstraint(
    int num_vars, double minimum_value_lower, double influence_value_offset,
    int max_num_values,
    std::function<AutoDiffVecXd(const Eigen::Ref<const AutoDiffVecXd>&, double)>
        value_function,
    std::function<VectorX<double>(const Eigen::Ref<const VectorX<double>>&,
                                  double)>
        value_function_double)
    : Constraint(1, num_vars, Vector1d(-kInf), Vector1d(1)),
      value_function_{std::move(value_function)},
      value_function_double_{std::move(value_function_double)},
      minimum_value_lower_{minimum_value_lower},
      influence_value_{influence_value_offset + minimum_value_lower},
      max_num_values_{max_num_values} {
  DRAKE_DEMAND(std::isfinite(minimum_value_lower_));
  DRAKE_DEMAND(std::isfinite(influence_value_offset));
  DRAKE_DEMAND(influence_value_offset > 0);
  this->set_penalty_function(QuadraticallySmoothedHingeLoss);
}

void MinimumValueLowerBoundConstraint::set_penalty_function(
    MinimumValuePenaltyFunction new_penalty_function) {
  set_penalty_function_impl(new_penalty_function, &penalty_function_,
                            &penalty_output_scaling_);
}

template <typename T>
void MinimumValueLowerBoundConstraint::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  // If we know that Values() will return at most zero values, then this
  // is a non-constraint. Set y=0 which trivially satisfies the constraint y
  // <= 1.
  const double y_for_empty_value = 0;
  EvalGeneric<T>(minimum_value_lower_, max_num_values_, y_for_empty_value,
                 influence_value_, value_function_double_, value_function_,
                 SmoothOverMax<double>, SmoothOverMax<T>, penalty_function_,
                 penalty_output_scaling_, x, y);
}

void MinimumValueLowerBoundConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DoEvalGeneric<double>(x, y);
}

void MinimumValueLowerBoundConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  DoEvalGeneric<AutoDiffXd>(x, y);
}

MinimumValueUpperBoundConstraint::MinimumValueUpperBoundConstraint(
    int num_vars, double minimum_value_upper, double influence_value_offset,
    int max_num_values,
    std::function<AutoDiffVecXd(const Eigen::Ref<const AutoDiffVecXd>&, double)>
        value_function,
    std::function<VectorX<double>(const Eigen::Ref<const VectorX<double>>&,
                                  double)>
        value_function_double)
    : Constraint(1, num_vars, Vector1d(1), Vector1d(kInf)),
      value_function_{std::move(value_function)},
      value_function_double_{std::move(value_function_double)},
      minimum_value_upper_{minimum_value_upper},
      influence_value_{influence_value_offset + minimum_value_upper},
      max_num_values_{max_num_values} {
  DRAKE_DEMAND(std::isfinite(minimum_value_upper_));
  DRAKE_DEMAND(std::isfinite(influence_value_offset));
  DRAKE_DEMAND(influence_value_offset > 0);
  this->set_penalty_function(QuadraticallySmoothedHingeLoss);
}

void MinimumValueUpperBoundConstraint::set_penalty_function(
    MinimumValuePenaltyFunction new_penalty_function) {
  set_penalty_function_impl(new_penalty_function, &penalty_function_,
                            &penalty_output_scaling_);
}

template <typename T>
void MinimumValueUpperBoundConstraint::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  // If we know that Values() will return at most zero values, then this
  // is a non-constraint. Set y=2 which always satisfies the constraint is y
  // >= 1.
  const double y_for_empty_value = 2;
  EvalGeneric<T>(minimum_value_upper_, max_num_values_, y_for_empty_value,
                 influence_value_, value_function_double_, value_function_,
                 SmoothUnderMax<double>, SmoothUnderMax<T>, penalty_function_,
                 penalty_output_scaling_, x, y);
}

void MinimumValueUpperBoundConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DoEvalGeneric<double>(x, y);
}

void MinimumValueUpperBoundConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  DoEvalGeneric<AutoDiffXd>(x, y);
}
}  // namespace solvers
}  // namespace drake
