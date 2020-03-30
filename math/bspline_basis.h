#pragma once

#include <algorithm>
#include <array>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"
#include "drake/math/knot_vector_type.h"

namespace drake {
namespace math {
/** Given a set of non-descending breakpoints t₀ ≤ t₁ ≤ ⋅⋅⋅ ≤ tₘ, a B-spline
basis of order k is a set of n + 1 (where n = m - k) piecewise polynomials of
degree k - 1 defined over those breakpoints. The elements of this set are
called "B-splines". The vector (t₀, t₁, ..., tₘ)' is referred to as
the "knot vector" of the basis and its elements are referred to as "knots".

A B-spline curve using a B-spline basis B, is a parametric curve mapping
parameter values in [tₖ₋₁, tₙ₊₁] to a vector space V. For t ∈ [tₖ₋₁, tₙ₊₁] the
value of the curve is given by the linear combination of n + 1 control points,
pᵢ ∈ V, with the elements of B evaluated at t.

For more information on B-splines and their uses, see (for example)
Patrikalakis et al. [1].

[1] https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node15.html */
template <typename T>
class BsplineBasis final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BsplineBasis);

  /** Constructs a B-spline basis with the specified `order` and `knots`.
  @pre `knots` is sorted in non-descending order.
  @throws std::invalid_argument if knots.size() < 2 * order. */
  BsplineBasis(int order, std::vector<T> knots);

  /** Constructs a B-spline basis with the specified `order` and
  `num_basis_functions` with an auto-generated knot vector. If `type ==
  KnotVectorType::kClampedUniform` the knot vector is a clamped uniform knot
  vector from 0 to 1. If `type == KnotVectorType::kUniform`, the knot vector
  is [-(order - 1), ..., num_basis_functions].
  @throws std::invalid_argument if num_basis_functions < order. */
  BsplineBasis(int order, int num_basis_functions,
               KnotVectorType type = KnotVectorType::kClampedUniform);

  /** The order of this B-spline basis (k in the class description). */
  int order() const { return order_; }

  /** The number of basis functions in this B-spline basis (n + 1 in the class
  description). */
  int num_basis_functions() const { return num_basis_functions_; }

  /** The knot vector of this B-spline basis (the vector (t₀, t₁, ..., tₘ)' in
  the class description). */
  const std::vector<T>& knots() const { return knots_; }

  /** The minimum allowable parameter value for B-spline curves using this
  basis (tₖ₋₁ in the class description). */
  const T& initial_parameter_value() const { return knots()[order() - 1]; }

  /** The maximum allowable parameter value for B-spline curves using this
  basis (tₙ₊₁ in the class description). */
  const T& final_parameter_value() const {
    return knots()[num_basis_functions()];
  }

  /** Returns the indices of the basis functions which evaluate to non-zero
  values for some parameter value in `parameter_interval`.
  @pre parameter_interval[0] ≤ parameter_interval[1]
  @pre parameter_interval[0] ≥ this->initial_parameter_value()
  @pre parameter_interval[1] ≤ this->final_parameter_value() */
  std::vector<int> ComputeActiveBasisFunctionIndices(
      const std::array<T, 2>& parameter_interval) const;

  /** Returns the indices of the basis functions which evaluate to non-zero
  values for `parameter_value`.
  @pre parameter_value ≥ this->initial_parameter_value()
  @pre parameter_value ≤ this->final_parameter_value() */
  std::vector<int> ComputeActiveBasisFunctionIndices(
      const T& parameter_value) const;

  /** If `parameter_value` ∈ [t[0], t[m]), return the knot index 𝑙such that
  t[𝑙] ≤ parameter_value < t[𝑙 + 1]. If `parameter_value` == t[m] return m - 1.
  @pre t[0] ≤ parameter_value ≤ t[m] */
  int FindContainingInterval(const T& parameter_value) const;

  /** Evaluates the B-spline curve defined by `this` and `control_points` at the
  given `parameter_value`.
  @param control_points Control points of the B-spline curve.
  @param parameter_value Parameter value at which to evaluate the B-spline
  curve defined by `this` and `control_points`.
  @pre control_points.size() == this->num_basis_functions()
  @pre parameter_value ≥ this->initial_parameter_value()
  @pre parameter_value ≤ this->final_parameter_value() */
  template <typename T_control_point>
  T_control_point EvaluateCurve(
      const std::vector<T_control_point>& control_points,
      T parameter_value) const {
    /* This function implements the de Boor algorithm. It uses the notation
    from Patrikalakis et al. [1]. Since the depth of recursion is known
    a-priori, the algorithm is flattened along the lines described in [2] to
    avoid duplicate computations.

     [1] https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node18.html
     [2] De Boor, Carl. "On calculating with B-splines." Journal of
         Approximation theory 6.1 (1972): 50-62.

    NOTE: The implementation of this method is included in the header so that
    it can be used with custom values of T_control_point. */
    DRAKE_DEMAND(static_cast<int>(control_points.size()) ==
                 num_basis_functions());
    DRAKE_DEMAND(parameter_value >= initial_parameter_value());
    DRAKE_DEMAND(parameter_value <= final_parameter_value());

    // Define short names to match notation in [1].
    const std::vector<T>& t = knots();
    const T t_bar = parameter_value;
    const int& k = order();

    // Find the knot index 𝑙 (ell in code) such that t[𝑙] ≤ t_bar < t[𝑙 + 1].
    // If t_bar == t[m] use m - 1.
    const int ell = FindContainingInterval(t_bar);

    // The vector that stores the intermediate de Boor points (the
    // pᵢʲ in [1]).
    std::vector<T_control_point> p(order());
    /* For j = 0, i goes from ell down to ell - (k - 1). Define r such that
    i = ell - r. */
    for (int r = 0; r < k; ++r) {
      const int i = ell - r;
      p.at(r) = control_points.at(i);
    }
    /* For j = 1, ..., k - 1, i goes from ell down to ell - (k - j - 1). Again,
    i = ell - r. */
    for (int j = 1; j < k; ++j) {
      for (int r = 0; r < k - j; ++r) {
        const int i = ell - r;
        // α = (t_bar - t[i]) / (t[i + k - j] - t[i]);
        T alpha = (t_bar - t.at(i)) / (t.at(i + order() - j) - t.at(i));
        p.at(r) = (1.0 - alpha) * p.at(r + 1) + alpha * p.at(r);
      }
    }
    return p.front();
  }

  /** Returns the value of the `i`-th basis function evaluated at
  `parameter_value`. */
  T EvaluateBasisFunctionI(int i, T parameter_value) const;

  bool operator==(const BsplineBasis& other) const;

 private:
  int order_;
  int num_basis_functions_;
  std::vector<T> knots_;

  bool IsControlPointActive(int control_point_index,
                            const std::array<T, 2>& parameter_interval) const;
};
}  // namespace math
}  // namespace drake
