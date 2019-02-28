#pragma once

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/hydroelastic_contact/triangle_quadrature_rule.h"

namespace drake {
namespace multibody {
namespace hydroelastic_contact {

/// A class for integrating a function using numerical quadrature over
/// triangular domains.
///
/// @tparam NumericReturnType the output type of the function being integrated.
///         Commonly will be a IEEE floating point scalar (e.g., `double`), but
///         could be an Eigen::VectorXd, a multibody::SpatialForce, or any
///         other numeric type that supports scalar multiplication and division
///         (i.e., operator*(const NumericReturnType&, double) and
///         operator/(const NumericReturnType&, double)) and addition with
///         another of the same type (i.e.,
///         operator+(const NumericReturnType&, const NumericReturnType&)).
/// @tparam T the scalar type of the function being integrated over. Supported
///         types are currently only IEEE floating point scalars.
template <typename NumericReturnType, typename T>
class TriangleQuadrature {
 public:
  /// Numerically integrates the function f over a triangle using the given
  /// quadrature rule and the initial value.
  /// @param f(p) a function that returns a numerical value for point p in the
  ///        domain of the triangle, specified in barycentric coordinates.
  ///        The barycentric coordinates are given by
  ///        (p[0], p[1], 1 - p[0] - p[1]).
  /// @param area the area of the triangle.
  /// @param initial_value the integral will be added to this value, which
  ///        you will typically want to set to zero.
  static NumericReturnType Integrate(
      std::function<NumericReturnType(const Vector2<T>&)> f,
      const TriangleQuadratureRule& rule,
      const T& area,
      const NumericReturnType& initial_value);
};

template <typename NumericReturnType, typename T>
NumericReturnType TriangleQuadrature<NumericReturnType, T>::Integrate(
    std::function<NumericReturnType(const Vector2<T>&)> f,
    const TriangleQuadratureRule& rule,
    const T& area,
    const NumericReturnType& initial_value) {

  // Initialize the integral.
  NumericReturnType integral = initial_value / area;

  // Get the quadrature points and weights.
  const std::vector<Eigen::Vector2d>& barycentric_coordinates =
      rule.quadrature_points();
  const std::vector<double>& weights = rule.weights();
  DRAKE_DEMAND(barycentric_coordinates.size() == weights.size());

  // Sum the weighted function evaluated at the transformed quadrature points.
  for (int i = 0; i < static_cast<int>(weights.size()); ++i)
    integral += f(barycentric_coordinates[i]) * weights[i];

  return integral * area;
}

}  // namespace hydroelastic_contact
}  // namespace multibody
}  // namespace drake
