#pragma once

#include <functional>

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
function is differentiable everywhere. The fomulation is described in
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
multidisciplinary workshop on Advances in preference handling. */
void QuadraticallySmoothedHingeLoss(double x, double* penalty,
                                    double* dpenalty_dx);

/** Constrain all elements of the vector returned by the user-provided function
to be no smaller than a specified minimum value.

The formulation of the constraint is

SmoothMax( φ((vᵢ - v_influence)/(v_influence - vₘᵢₙ)) / φ(-1) ) ≤ 1

where vᵢ is the i-th value returned by the user-provided function, vₘᵢₙ is the
minimum allowable value, v_influence is the "influence value" (the value below
which an element influences the constraint or, conversely, the value above which
an element is ignored), φ is a solvers::MinimumValuePenaltyFunction, and
SmoothMax(v) is a smooth, conservative approximation of max(v) (i.e.
SmoothMax(v) >= max(v), for all v). We require that vₘᵢₙ < v_influence. The
input scaling (vᵢ - v_influence)/(v_influence - vₘᵢₙ) ensures that at the
boundary of the feasible set (when vᵢ == vₘᵢₙ), we evaluate the penalty function
at -1, where it is required to have a non-zero gradient. The user-provided
function may return a vector with up to `max_num_values` elements. If it returns
a vector with fewer than `max_num_values` elements, the remaining elements are
assumed to be greater than the "influence value". */
class MinimumValueConstraint final : public solvers::Constraint {
 public:
  /** Constructs a MinimumValueConstraint.
  @param num_vars The number of inputs to `value_function`
  @param minimum_value The minimum allowed value, vₘᵢₙ, for all elements of the
  vector returned by `value_function`.
  @param influence_value_offset The difference between the
  influence value, v_influence, and the minimum value, vₘᵢₙ (see class
  documentation). This value must be finite and strictly positive, as it is used
  to scale the values returned by `value_function`. Smaller values may
  decrease the amount of computation required for each constraint evaluation if
  `value_function` can quickly determine that some elements will be
  larger than the influence value and skip the computation associated with those
  elements. @default 1
  @param max_num_values The maximum number of elements in the vector returned by
  `value_function`.
  @param value_function User-provided function that takes a `num_vars`-element
  vector and the influence distance as inputs and returns a vector with up to
  `max_num_values` elements. The function can omit from the return vector any
  elements larger than the provided influence distance.
  @param value_function_double Optional user-provide function that computes the
  same values as `value_function` but for double rather than AutoDiffXd. If
  ommited, `value_function` will be called (and the gradients discarded) when
  this constraint is evaluated for doubles.
  @pre `value_function_double(math::autoDiffToValueMatrix(x), v_influence) ==
  math::autoDiffToValueMatrix(value_function(x, v_influence))` for all x.
  @pre `value_function(x).size() <= max_num_values` for all x.
  @throws std::invalid_argument if influence_value_offset = ∞.
  @throws std::invalid_argument if influence_value_offset ≤ 0.
  */
  MinimumValueConstraint(
      int num_vars, double minimum_value, double influence_value_offset,
      int max_num_values,
      std::function<AutoDiffVecXd(const Eigen::Ref<const AutoDiffVecXd>&,
                                  double)>
          value_function,
      std::function<VectorX<double>(const Eigen::Ref<const VectorX<double>>&,
                                    double)>
          value_function_double = {});

  ~MinimumValueConstraint() override {}

  /** Getter for the minimum value. */
  double minimum_value() const { return minimum_value_; }

  /** Getter for the influence value. */
  double influence_value() const { return influence_value_; }

  /** Getter for maximum number of values expected from value_function. */
  double max_num_values() const { return max_num_values_; }

  /** Setter for the penalty function. */
  void set_penalty_function(MinimumValuePenaltyFunction new_penalty_function);

 private:
  template <typename T>
  VectorX<T> Values(const Eigen::Ref<const VectorX<T>>& x) const;

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
        "MinimumValueConstraint::DoEval() does not work for symbolic "
        "variables.");
  }

  std::function<AutoDiffVecXd(const AutoDiffVecXd&, double)> value_function_;
  std::function<VectorX<double>(const VectorX<double>&, double)>
      value_function_double_;
  const double minimum_value_;
  const double influence_value_;
  /** Stores the value of
  1 / φ((vₘᵢₙ - v_influence)/(v_influence - vₘᵢₙ)) = 1 / φ(-1). This is
  used to scale the output of the penalty function to be 1 when v == vₘᵢₙ. */
  double penalty_output_scaling_;
  int max_num_values_{};
  MinimumValuePenaltyFunction penalty_function_{};
};

}  // namespace solvers
}  // namespace drake
