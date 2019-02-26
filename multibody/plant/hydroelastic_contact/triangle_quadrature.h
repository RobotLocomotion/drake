#pragma once

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/hydroelastic_contact/triangle_quadrature_rule.h"

namespace drake {
namespace multibody {
namespace hydroelastic_contact {

template <typename NumericReturnType, typename T>
class TriangleQuadrature {
 public:
  /// Numerically integrates the function f over the triangle defined by
  /// `vertex_a`, `vertex_b`, and `vertex_c`, using the given quadrature rule
  /// and the initial value.
  /// @param f a function that returns a numerical value for a point in the
  ///        domain of the triangle.
  /// @param initial_value the integral will be added to this value, which
  ///        you will typically want to set to zero.
  static NumericReturnType Integrate(
      const Vector2<T>& vertex_a,
      const Vector2<T>& vertex_b,
      const Vector2<T>& vertex_c,
      std::function<NumericReturnType(const Vector2<T>&)> f,
      const TriangleQuadratureRule& rule,
      const NumericReturnType& initial_value);

 private:
  // Quadrature rules are defined for the "canonical triangle" (0, 0), (1, 0),
  // (0, 1). This function transforms the quadrature points from the barycentric
  // coordinates of the canonical triangle to Cartesian coordinates as
  // described on pp. 10-12 of:
  // http://math2.uncc.edu/~shaodeng/TEACHING/math5172/Lectures/Lect_15.PDF
  static Vector2<T> TransformBarycentricToCartesianCoordinates(
      const Vector2<T>& a, const Vector2<T>& b, const Vector2<T>& c,
      const Vector2<T>& quadrature_point) {
    Vector2<T> canonical_point;
    canonical_point[0] =
        a[0] * (1 - quadrature_point[0] - quadrature_point[1]) +
        b[0] * quadrature_point[0] + c[0] * quadrature_point[1];
    canonical_point[1] =
        a[1] * (1 - quadrature_point[0] - quadrature_point[1]) +
        b[1] * quadrature_point[0] + c[1] * quadrature_point[1];
    return canonical_point;
  }
};

template <typename NumericReturnType, typename T>
NumericReturnType TriangleQuadrature<NumericReturnType, T>::Integrate(
    const Vector2<T>& a, const Vector2<T>& b, const Vector2<T>& c,
    std::function<NumericReturnType(const Vector2<T>&)> f,
    const TriangleQuadratureRule& rule,
    const NumericReturnType& initial_value) {
  using std::abs;

  // Calculate the area of the triangle.
  const T area = abs(a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) +
      c[0] * (a[1] - b[1])) / 2.0;

  // Initialize the integral.
  NumericReturnType integral = initial_value;

  // Get the quadrature points and weights.
  const std::vector<Eigen::Vector2d>& points = rule.quadrature_points();
  const std::vector<double>& weights = rule.weights();

  // Sum the weighted function evaluated at the transformed quadrature points.
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    integral += f(TransformBarycentricToCartesianCoordinates(
        a, b, c, points[i])) * weights[i];
  }

  return integral * area;
}

}  // namespace hydroelastic_contact
}  // namespace multibody
}  // namespace drake
