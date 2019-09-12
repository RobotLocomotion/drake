#pragma once

#include <algorithm>
#include <array>
#include <vector>

#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"
#include "drake/math/knot_vector_type.h"

namespace drake {
namespace math {
template <typename T>
class BsplineBasis final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BsplineBasis);

  /** Constructs a B-spline basis with the specified `order` and `knots`.
  @pre `knots` is sorted in ascending order.
  @throws std::invalid_argument if knots.size() < 2 * order. */
  BsplineBasis(int order, std::vector<T> knots);

  /** Constructs a B-spline basis with the specified `order` and
  `num_control_points` with an auto-generated knot vector. If `type ==
  KnotVectorType::kClampedUniform` the knot vector is a clamped uniform knot
  vector from 0 to 1. If `type == KnotVectorType::kUniform`, the knot vector
  is [-(order - 1), ..., num_control_points].
  @throws std::invalid_argument if num_control_points < order. */
  BsplineBasis(int order, int num_control_points,
               KnotVectorType type = KnotVectorType::kClampedUniform);

  int order() const { return order_; }

  int num_control_points() const { return num_control_points_; }

  const std::vector<T>& knots() const { return knots_; }

  T initial_parameter_value() const { return knots()[order() - 1]; }

  T final_parameter_value() const { return knots()[num_control_points()]; }

  bool IsControlPointActive(int control_point_index,
                            const std::array<T, 2>& parameter_interval) const;

  bool IsControlPointActive(int control_point_index,
                            const T& parameter_value) const;

  std::vector<int> ComputeActiveControlPointIndices(
      const std::array<T, 2>& parameter_interval) const;

  std::vector<int> ComputeActiveControlPointIndices(
      const T& parameter_value) const;

  /** Find the knot index 𝑙 (ell in code) such that t[𝑙] ≤ t_bar < t[𝑙 + 1]. */
  int FindContainingInterval(const T& parameter_value) const;

  /** Implements the de Boor algorithm. Uses the notation from Patrikalakis et
  al. [1]. Since the depth of recursion is known a-priori, the algorithm is
  flattened along the lines described in [2] to avoid duplicate computations.

   [1] https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node18.html
   [2] De Boor, Carl. "On calculating with B-splines." Journal of
       Approximation theory 6.1 (1972): 50-62.
  @param control_points Control points of a B-spline curve. In the notation
  of [1] this is p.
  @param parameter_value Parameter value at which to evaluate the B-spline
  curve defined by `this` and `control_points`. In the notation of [1] this
  is \bar{t}. */
  template <typename T_control_point>
  T_control_point DeBoor(const std::vector<T_control_point>& control_points,
                         T parameter_value) const {
    // NOTE: The implementation of this method is included in the header so that
    // it can be used with custom values of T_control_point.
    using std::max;
    using std::min;
    // Define short names to match notation in [1].
    const std::vector<T>& t = knots();
    const T t_bar = min(max(parameter_value, initial_parameter_value()),
                        final_parameter_value());
    const int& k = order();

    // Find the knot index 𝑙 (ell in code) such that t[𝑙] ≤ t_bar < t[𝑙 + 1].
    const int ell = FindContainingInterval(t_bar);

    // The vector that stores the intermediate de Boor points (the
    // pᵢʲ in [1]).
    std::vector<T_control_point> p(order());
    // For j = 0, i goes from ell down to ell - (k - 1). Define r such that
    // i = ell - r.
    for (int r = 0; r < k; ++r) {
      const int i = ell - r;
      p.at(r) = control_points.at(i);
    }
    // For j = 1, ..., k - 1, i goes from ell down to ell - (k - j - 1). Again,
    // i = ell - r.
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

  T BasisFunctionValue(int index, T parameter_value) const;

  bool operator==(const BsplineBasis& other) const;

 private:
  int order_;
  int num_control_points_;
  std::vector<T> knots_;
};
}  // namespace math
}  // namespace drake
