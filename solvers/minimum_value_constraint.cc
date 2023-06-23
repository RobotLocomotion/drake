#include "drake/solvers/minimum_value_constraint.h"

#include <limits>
#include <vector>

#include "drake/math/autodiff_gradient.h"
#include "drake/math/soft_min_max.h"

namespace drake {
namespace solvers {
const double kInf = std::numeric_limits<double>::infinity();

namespace {
/** Computes a smooth over approximation of max(x). */
template <typename T>
T SmoothOverMax(const std::vector<T>& x) {
  // We compute the smooth max of x as smoothmax(x) = log(∑ᵢ exp(αxᵢ)) / α.
  // This smooth max approaches max(x) as α increases. We choose α = 100, as
  // that gives a qualitatively good fit for xᵢ ∈ [0, 1], which is the range of
  // potential penalty values when the MinimumValueConstraint is feasible.
  return math::SoftOverMax(x, 100 /* alpha */);
}

/** Computes a smooth over approximation of max(x). */
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
    : MinimumValueConstraint(
          num_vars, minimum_value, kInf, minimum_value + influence_value_offset,
          max_num_values, value_function, value_function_double) {}

namespace {
int NumConstraints(double minimum_value_lower, double minimum_value_upper) {
  return static_cast<int>(std::isfinite(minimum_value_lower)) +
         static_cast<int>(std::isfinite(minimum_value_upper));
}

Eigen::VectorXd LowerBounds(double minimum_value_lower,
                            double minimum_value_upper) {
  if (std::isfinite(minimum_value_lower) &&
      std::isfinite(minimum_value_upper)) {
    return Eigen::Vector2d(-kInf, 1);
  } else if (std::isfinite(minimum_value_lower)) {
    return Vector1d(-kInf);
  } else if (std::isfinite(minimum_value_upper)) {
    return Vector1d(1);
  } else {
    throw std::runtime_error(
        "MinimumValueConstraint: at least one of minimum_value_lower and "
        "minimum_value_upper should be finite.");
  }
}

Eigen::VectorXd UpperBounds(double minimum_value_lower,
                            double minimum_value_upper) {
  if (std::isfinite(minimum_value_lower) &&
      std::isfinite(minimum_value_upper)) {
    return Eigen::Vector2d(1, kInf);
  } else if (std::isfinite(minimum_value_lower)) {
    return Vector1d(1);
  } else if (std::isfinite(minimum_value_upper)) {
    return Vector1d(kInf);
  } else {
    throw std::runtime_error(
        "MinimumValueConstraint: at least one of minimum_value_lower and "
        "minimum_value_upper should be finite.");
  }
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
      value_function_{value_function},
      value_function_double_{value_function_double},
      minimum_value_lower_{minimum_value_lower},
      minimum_value_upper_{minimum_value_upper},
      influence_value_{influence_value},
      max_num_values_{max_num_values} {
  DRAKE_DEMAND(influence_value_ > minimum_value_lower_);
  if (std::isfinite(minimum_value_upper)) {
    DRAKE_DEMAND(influence_value_ > minimum_value_upper_);
  }
  DRAKE_DEMAND(std::isfinite(influence_value_));
  set_penalty_function(QuadraticallySmoothedHingeLoss);
}

void MinimumValueConstraint::set_penalty_function(
    MinimumValuePenaltyFunction new_penalty_function) {
  penalty_function_ = new_penalty_function;
  double unscaled_penalty_at_minimum_value{};
  penalty_function_(-1, &unscaled_penalty_at_minimum_value, nullptr);
  penalty_output_scaling_ = 1 / unscaled_penalty_at_minimum_value;
}

template <>
VectorX<double> MinimumValueConstraint::Values(
    const Eigen::Ref<const VectorX<double>>& x) const {
  return value_function_double_ ? value_function_double_(x, influence_value_)
                                : math::ExtractValue(value_function_(
                                      x.cast<AutoDiffXd>(), influence_value_));
}

template <>
AutoDiffVecXd MinimumValueConstraint::Values(
    const Eigen::Ref<const AutoDiffVecXd>& x) const {
  return value_function_(x, influence_value_);
}

template <typename T>
void MinimumValueConstraint::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  y->resize(num_constraints());

  // Index for the constraint
  // SoftOverMax( φ((vᵢ - v_influence)/(v_influence - vₘᵢₙ_lower)) / φ(-1) ) ≤ 1
  // Use -1 if minimum_value_lower_ is inf.
  const int y_lower_index = std::isfinite(minimum_value_lower_) ? 0 : -1;
  // Index for the constraint
  // SoftUnderMax( φ((vᵢ - v_influence)/(v_influence - vₘᵢₙ_upper)) / φ(-1) ) ≥
  // 1 Use -1 if minimum_value_upper_ is inf.
  const int y_upper_index =
      std::isfinite(minimum_value_upper_) ? y_lower_index + 1 : -1;

  // If we know that Values() will return at most zero values, then this
  // is a non-constraint. Return zero for the lower bound constraint, and 2 for
  // the upper bound constraint.
  if (max_num_values_ == 0) {
    if (y_lower_index >= 0) {
      InitializeY(x, y, y_lower_index, 0.0);
    }
    if (y_upper_index >= 0) {
      InitializeY(x, y, y_upper_index, 2.0);
    }
    return;
  }

  // Initialize y(y_lower_index) and y(y_upper_index) to
  // SmoothOverMax([0, 0, ..., 0]).
  if (y_lower_index >= 0) {
    InitializeY(x, y, y_lower_index,
                SmoothOverMax(std::vector<double>(max_num_values_, 0.0)));
  }
  if (y_upper_index >= 0) {
    InitializeY(x, y, y_upper_index,
                SmoothUnderMax(std::vector<double>(max_num_values_, 0.0)));
  }

  VectorX<T> values = Values(x);
  const int num_values = static_cast<int>(values.size());
  DRAKE_ASSERT(num_values <= max_num_values_);
  if (y_lower_index >= 0) {
    std::vector<T> penalties_lower{};
    penalties_lower.reserve(max_num_values_);
    for (int i = 0; i < num_values; ++i) {
      const T& value = values(i);
      if (value < influence_value_) {
        penalties_lower.emplace_back();
        Penalty(value, minimum_value_lower_, influence_value_,
                penalty_function_, &penalties_lower.back());
        penalties_lower.back() *= penalty_output_scaling_;
      }
    }
    if (!penalties_lower.empty()) {
      // Pad penalties up to max_num_values_ so that the constraint
      // function is actually smooth.
      penalties_lower.resize(max_num_values_, T{0.0});
      (*y)(y_lower_index) = SmoothOverMax(penalties_lower);
    }
  }
  if (y_upper_index >= 0) {
    std::vector<T> penalties_upper{};
    penalties_upper.reserve(max_num_values_);
    for (int i = 0; i < num_values; ++i) {
      const T& value = values(i);
      if (value < influence_value_) {
        penalties_upper.emplace_back();
        Penalty(value, minimum_value_upper_, influence_value_,
                penalty_function_, &penalties_upper.back());
        penalties_upper.back() *= penalty_output_scaling_;
      }
    }
    if (!penalties_upper.empty()) {
      // Pad penalties up to max_num_values_ so that the constraint
      // function is actually smooth.
      penalties_upper.resize(max_num_values_, T{0.0});
      (*y)(y_upper_index) = SmoothUnderMax(penalties_upper);
    }
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
