#pragma once

#include <functional>
#include <utility>
#include <vector>

#include "drake/solvers/constraint.h"

namespace drake {
namespace solvers {
/** Computes the penalty function φ(x) and its derivatives dφ(x)/dx. Valid
penalty functions must meet the following criteria:

1.     φ(x) ≥ 0 ∀ x ∈ ℝ.
2. dφ(x)/dx ≤ 0 ∀ x ∈ ℝ.
3.     φ(x) = 0 ∀ x ≥ 0.
4. dφ(x)/dx < 0 ∀ x < 0.

If `dpenalty_dx` is nullptr, the function should only compute φ(x). */
using MinimumValuePenaltyFunction =
    std::function<void(double x, double* penalty, double* dpenalty_dx)>;

/** A hinge loss function smoothed by exponential function. This loss
function is differentiable everywhere. The formulation is described in
section II.C of [2].
The penalty is
<pre class="unicode-art">
       ⎧ 0            if x ≥ 0
φ(x) = ⎨
       ⎩ -x exp(1/x)  if x < 0.
</pre>
[2] "Whole-body Motion Planning with Centroidal Dynamics and Full
Kinematics" by Hongkai Dai, Andres Valenzuela and Russ Tedrake, IEEE-RAS
International Conference on Humanoid Robots, 2014. */
void ExponentiallySmoothedHingeLoss(double x, double* penalty,
                                    double* dpenalty_dx);

/** A linear hinge loss, smoothed with a quadratic loss near the origin. The
formulation is in equation (6) of [1].
The penalty is
<pre class="unicode-art">
       ⎧  0        if x ≥ 0
φ(x) = ⎨  x²/2     if -1 < x < 0
       ⎩  -0.5 - x if x ≤ -1.
</pre>
[1] "Loss Functions for Preference Levels: Regression with Discrete Ordered
Labels." by Jason Rennie and Nathan Srebro, Proceedings of IJCAI
multidisciplinary workshop on Advances in Preference Handling. */
void QuadraticallySmoothedHingeLoss(double x, double* penalty,
                                    double* dpenalty_dx);

/** Constrain min(v) >= lb where v=f(x). Namely all elements of the
vector `v` returned by the user-provided function f(x) to be no smaller than a
specified value `lb`.

The formulation of the constraint is

<pre>
SmoothOverMax( φ((vᵢ - v_influence)/(v_influence - lb)) / φ(-1) ) ≤ 1
</pre>

where vᵢ is the i-th value returned by the user-provided function, `lb` is
the minimum allowable value. v_influence is the "influence value" (the
value below which an element influences the constraint or, conversely, the
value above which an element is ignored), φ is a
solvers::MinimumValuePenaltyFunction, and SmoothOverMax(v) is a smooth, over
approximation of max(v) (i.e. SmoothOverMax(v) >= max(v), for all v). We
require that lb < v_influence. The input scaling (vᵢ -
v_influence)/(v_influence - lb) ensures that at the boundary of the feasible
set (when vᵢ == lb), we evaluate the penalty function at -1, where it is
required to have a non-zero gradient. The user-provided function may return a
vector with up to `max_num_values` elements. If it returns a vector with fewer
than `max_num_values` elements, the remaining elements are assumed to be
greater than the "influence value". */
class MinimumValueLowerBoundConstraint final : public solvers::Constraint {
 public:
  /** Constructs a MinimumValueLowerBoundConstraint.
  min(v) >= lb
  And we set ub to infinity in min(v) <= ub.

  @param num_vars The number of inputs to `value_function`
  @param minimum_value_lower The minimum allowed value, lb, for all elements
  of the vector returned by `value_function`.
  @param influence_value_offset The difference between the
  influence value, v_influence, and the minimum value, lb (see class
  documentation). This value must be finite and strictly positive, as it is
  used to scale the values returned by `value_function`. Smaller values may
  decrease the amount of computation required for each constraint evaluation
  if `value_function` can quickly determine that some elements will be larger
  than the influence value and skip the computation associated with those
  elements.
  @param max_num_values The maximum number of elements in the vector returned
  by `value_function`.
  @param value_function User-provided function that takes a `num_vars`-element
  vector and the influence distance as inputs and returns a vector with up to
  `max_num_values` elements. The function can omit from the return vector any
  elements larger than the provided influence distance.
  @param value_function_double Optional user-provide function that computes
  the same values as `value_function` but for double rather than AutoDiffXd.
  If omitted, `value_function` will be called (and the gradients discarded)
  when this constraint is evaluated for doubles.
  @pre `value_function_double(ExtractValue(x), v_influence) ==
  ExtractValue(value_function(x, v_influence))` for all x.
  @pre `value_function(x).size() <= max_num_values` for all x.
  @throws std::exception if influence_value_offset = ∞.
  @throws std::exception if influence_value_offset ≤ 0.
  */
  MinimumValueLowerBoundConstraint(
      int num_vars, double minimum_value_lower, double influence_value_offset,
      int max_num_values,
      std::function<AutoDiffVecXd(const Eigen::Ref<const AutoDiffVecXd>&,
                                  double)>
          value_function,
      std::function<Eigen::VectorXd(const Eigen::Ref<const Eigen::VectorXd>&,
                                    double)>
          value_function_double = {});

  ~MinimumValueLowerBoundConstraint() override;

  /** Getter for the lower bound on the minimum value. */
  double minimum_value_lower() const { return minimum_value_lower_; }

  /** Getter for the influence value. */
  double influence_value() const { return influence_value_; }

  /** Getter for maximum number of values expected from value_function. */
  int max_num_values() const { return max_num_values_; }

  /** Setter for the penalty function. */
  void set_penalty_function(MinimumValuePenaltyFunction new_penalty_function);

 private:
  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "MinimumValueLowerBoundConstraint::DoEval() does not work for "
        "symbolic "
        "variables.");
  }

  std::function<AutoDiffVecXd(const AutoDiffVecXd&, double)> value_function_;
  std::function<VectorX<double>(const VectorX<double>&, double)>
      value_function_double_;
  const double minimum_value_lower_;
  const double influence_value_;
  /** Stores the value of
  1 / φ((vₘᵢₙ - v_influence)/(v_influence - vₘᵢₙ)) = 1 / φ(-1). This is
  used to scale the output of the penalty function to be 1 when v == vₘᵢₙ. */
  double penalty_output_scaling_;
  const int max_num_values_{};
  MinimumValuePenaltyFunction penalty_function_{};
};

/** Constrain min(v) <= ub where v=f(x). Namely at least one element of the
vector `v` returned by the user-provided function f(x) to be no larger than a
specified value `ub`.

The formulation of the constraint is

<pre>
SmoothUnderMax( φ((vᵢ - v_influence)/(v_influence - ub)) / φ(-1) ) ≥ 1
</pre>

where vᵢ is the i-th value returned by the user-provided function, `ub` is the
upper bound for the min(v). (Note that `ub` is NOT the upper bound of `v`).
v_influence is the "influence value" (the value below which an element
influences the constraint or, conversely, the value above which an element is
ignored), φ is a solvers::MinimumValuePenaltyFunction. SmoothUnderMax(x) is a
smooth, under approximation of max(v) (i.e. SmoothUnderMax(v) <= max(v) for
all v). We require that ub < v_influence. The input scaling (vᵢ -
v_influence)/(v_influence - ub) ensures that at the boundary of the feasible
set (when vᵢ == ub), we evaluate the penalty function at -1, where it is
required to have a non-zero gradient. The user-provided function may return a
vector with up to `max_num_values` elements. If it returns a vector with fewer
than `max_num_values` elements, the remaining elements are assumed to be
greater than the "influence value". */
class MinimumValueUpperBoundConstraint final : public solvers::Constraint {
 public:
  /** Constructs a MinimumValueUpperBoundConstraint.
  min(v) <= ub

  @param num_vars The number of inputs to `value_function`
  @param minimum_value_upper The upper bound on the minimum allowed value for
  all elements of the vector returned by `value_function`, namely
  min(value_function(x)) <= minimum_value_upper
  @param influence_value_offset The difference between the
  influence value, v_influence, and minimum_value_upper. This value must be
  finite and strictly positive, as it is used to scale the values returned by
  `value_function`. Larger values may increase the possibility of finding a
  solution to the constraint. With a small v_influence, the value_function
  will ignore the entries with value less than v_influence. While it is
  possible that by changing x, that value (that currently been ignored) can
  decrease to below ub with a different x, by using a small v_influence, the
  gradient of that entry is never considered if the entry is ignored. We
  strongly suggest using a larger `v_influence` compared to the one used in
  MinimumValueConstraint when constraining min(v) >= lb.
  @param max_num_values The maximum number of elements in the vector returned
  by `value_function`.
  @param value_function User-provided function that takes a `num_vars`-element
  vector and the influence distance as inputs and returns a vector with up to
  `max_num_values` elements. The function can omit from the return vector any
  elements larger than the provided influence distance.
  @param value_function_double Optional user-provide function that computes
  the same values as `value_function` but for double rather than AutoDiffXd.
  If omitted, `value_function` will be called (and the gradients discarded)
  when this constraint is evaluated for doubles.
  @pre `value_function_double(ExtractValue(x), v_influence) ==
  ExtractValue(value_function(x, v_influence))` for all x.
  @pre `value_function(x).size() <= max_num_values` for all x.
  @throws std::exception if influence_value_offset = ∞.
  @throws std::exception if influence_value_offset ≤ 0.
  */
  MinimumValueUpperBoundConstraint(
      int num_vars, double minimum_value_upper, double influence_value_offset,
      int max_num_values,
      std::function<AutoDiffVecXd(const Eigen::Ref<const AutoDiffVecXd>&,
                                  double)>
          value_function,
      std::function<Eigen::VectorXd(const Eigen::Ref<const Eigen::VectorXd>&,
                                    double)>
          value_function_double = {});

  ~MinimumValueUpperBoundConstraint() override;

  /** Getter for the upper bound on the minimum value. */
  double minimum_value_upper() const { return minimum_value_upper_; }

  /** Getter for the influence value. */
  double influence_value() const { return influence_value_; }

  /** Getter for maximum number of values expected from value_function. */
  int max_num_values() const { return max_num_values_; }

  /** Setter for the penalty function. */
  void set_penalty_function(MinimumValuePenaltyFunction new_penalty_function);

 private:
  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "MinimumValueUpperBoundConstraint::DoEval() does not work for "
        "symbolic "
        "variables.");
  }

  std::function<AutoDiffVecXd(const AutoDiffVecXd&, double)> value_function_;
  std::function<VectorX<double>(const VectorX<double>&, double)>
      value_function_double_;
  const double minimum_value_upper_;
  const double influence_value_;
  /** Stores the value of
  1 / φ((vₘᵢₙ - v_influence)/(v_influence - vₘᵢₙ)) = 1 / φ(-1). This is
  used to scale the output of the penalty function to be 1 when v == vₘᵢₙ. */
  double penalty_output_scaling_;
  const int max_num_values_{};
  MinimumValuePenaltyFunction penalty_function_{};
};

}  // namespace solvers
}  // namespace drake
