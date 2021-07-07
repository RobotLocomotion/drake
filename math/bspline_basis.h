#pragma once

#include <array>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/name_value.h"
#include "drake/math/knot_vector_type.h"

namespace drake {
namespace math {
/** Given a set of non-descending breakpoints t₀ ≤ t₁ ≤ ⋅⋅⋅ ≤ tₘ, a B-spline
basis of order k is a set of n + 1 (where n = m - k) piecewise polynomials of
degree k - 1 defined over those breakpoints. The elements of this set are
called "B-splines". The vector (t₀, t₁, ..., tₘ)' is referred to as
the "knot vector" of the basis and its elements are referred to as "knots".

At a breakpoint with multiplicity p (i.e. a breakpoint that appears p times in
the knot vector), B-splines are guaranteed to have Cᵏ⁻ᵖ⁻¹ continuity.

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

  BsplineBasis() : BsplineBasis<T>(0, {}) {}

  /** Constructs a B-spline basis with the specified `order` and `knots`.
  @pre `knots` is sorted in non-descending order.
  @throws std::exception if knots.size() < 2 * order. */
  BsplineBasis(int order, std::vector<T> knots);

  /** Constructs a B-spline basis with the specified `order`,
  `num_basis_functions`, `initial_parameter_value`, `final_parameter_value`,
  and an auto-generated knot vector of the specified `type`.
  @throws std::exception if num_basis_functions < order
  @pre initial_parameter_value ≤ final_parameter_value */
  BsplineBasis(int order, int num_basis_functions,
               KnotVectorType type = KnotVectorType::kClampedUniform,
               const T& initial_parameter_value = 0,
               const T& final_parameter_value = 1);

#ifdef DRAKE_DOXYGEN_CXX
  /** Conversion constructor. Constructs an instance of BsplineBasis<T> from a
  double-valued basis. */
  explicit BsplineBasis(const BsplineBasis<double>& other);
#else
  template <typename U = T>
  explicit BsplineBasis(const BsplineBasis<double>& other,
               /* Prevents ambiguous declarations between default copy
               constructor on double and conversion constructor on T = double.
               The conversion constructor for T = double will fail to be
               instantiated because the second, "hidden" parameter will fail to
               be defined for U = double. */
               typename std::enable_if_t<!std::is_same_v<U, double>>* = {})
      : order_(other.order()) {
    knots_.reserve(other.knots().size());
    for (const auto& knot : other.knots()) {
      knots_.push_back(T(knot));
    }
  }
#endif

  /** The order of this B-spline basis (k in the class description). */
  int order() const { return order_; }

  /** The degree of the piecewise polynomials comprising this B-spline basis
  (k - 1 in the class description). */
  int degree() const { return order() - 1; }

  /** The number of basis functions in this B-spline basis (n + 1 in the class
  description). */
  int num_basis_functions() const { return knots_.size() - order_; }

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

  /** For a `parameter_value` = t, the interval that contains it is the pair of
  knot values [tᵢ, tᵢ₊₁] for the greatest i such that tᵢ ≤ t and
  tᵢ < final_parameter_value(). This function returns that value of i.
  @pre parameter_value ≥ initial_parameter_value()
  @pre parameter_value ≤ final_parameter_value() */
  int FindContainingInterval(const T& parameter_value) const;

  /** Returns the indices of the basis functions which may evaluate to non-zero
  values for some parameter value in `parameter_interval`; all other basis
  functions are strictly zero over `parameter_interval`.
  @pre parameter_interval[0] ≤ parameter_interval[1]
  @pre parameter_interval[0] ≥ initial_parameter_value()
  @pre parameter_interval[1] ≤ final_parameter_value() */
  std::vector<int> ComputeActiveBasisFunctionIndices(
      const std::array<T, 2>& parameter_interval) const;

  /** Returns the indices of the basis functions which may evaluate to non-zero
  values for `parameter_value`; all other basis functions are strictly zero at
  this point.
  @pre parameter_value ≥ initial_parameter_value()
  @pre parameter_value ≤ final_parameter_value() */
  std::vector<int> ComputeActiveBasisFunctionIndices(
      const T& parameter_value) const;

  /** Evaluates the B-spline curve defined by `this` and `control_points` at the
  given `parameter_value`.
  @param control_points Control points of the B-spline curve.
  @param parameter_value Parameter value at which to evaluate the B-spline
  curve defined by `this` and `control_points`.
  @pre control_points.size() == num_basis_functions()
  @pre parameter_value ≥ initial_parameter_value()
  @pre parameter_value ≤ final_parameter_value() */
  template <typename T_control_point>
  T_control_point EvaluateCurve(
      const std::vector<T_control_point>& control_points,
      const T& parameter_value) const {
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
    const T& t_bar = parameter_value;
    const int k = order();

    /* Find the index, 𝑙, of the greatest knot that is less than or equal to
    t_bar and strictly less than final_parameter_value(). */
    const int ell = FindContainingInterval(t_bar);
    // The vector that stores the intermediate de Boor points (the pᵢʲ in [1]).
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
        const T alpha = (t_bar - t.at(i)) / (t.at(i + k - j) - t.at(i));
        p.at(r) = (1.0 - alpha) * p.at(r + 1) + alpha * p.at(r);
      }
    }
    return p.front();
  }

  /** Returns the value of the `i`-th basis function evaluated at
  `parameter_value`. */
  T EvaluateBasisFunctionI(int i, const T& parameter_value) const;

  boolean<T> operator==(const BsplineBasis& other) const;

  boolean<T> operator!=(const BsplineBasis& other) const;

  /** Passes this object to an Archive; see @ref serialize_tips for background.
  This method is only available when T = double. */
  template <typename Archive>
#ifdef DRAKE_DOXYGEN_CXX
  void
#else
  // Restrict this method to T = double only; we must mix "Archive" into the
  // conditional type for SFINAE to work, so we just check it against void.
  std::enable_if_t<std::is_same_v<T, double> && !std::is_void_v<Archive>>
#endif
  Serialize(Archive* a) {
    a->Visit(MakeNameValue("order", &order_));
    a->Visit(MakeNameValue("knots", &knots_));
    DRAKE_THROW_UNLESS(CheckInvariants());
  }

 private:
  bool CheckInvariants() const;

  int order_{};
  std::vector<T> knots_;
};
}  // namespace math
}  // namespace drake
