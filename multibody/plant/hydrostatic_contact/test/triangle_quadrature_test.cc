#include <gtest/gtest.h>

#include "drake/multibody/plant/hydrostatic_contact/triangle_quadrature.h"
#include "drake/multibody/plant/hydrostatic_contact/gaussian_triangle_quadrature_rule.h"

using Vector2d = drake::Vector2<double>;
using drake::multibody::hydrostatic::TriangleQuadrature;
using drake::multibody::hydrostatic::GaussianTriangleQuadratureRule;

GTEST_TEST(TriangleQuadrature, FirstOrder) {
  // Function to be integrated: 5
  auto f = [](const Vector2d& p) -> double {
    return 5;
  };

  // The triangle vertices, constructed to give us a unit area triangle.
  Vector2d v[3] = { {0.0, 0.0}, {2.0, 0.0}, {2.0, 1.0} };

  // Test Gaussian quadrature rules of orders 1 through 5.
  for (int order = 1; order <= 5; ++order) {
    GaussianTriangleQuadratureRule rule(order);

    // Compute the integral.
    double result = TriangleQuadrature<double, double>::Integrate(
        v[0], v[1], v[2], f, rule, 0.0);
    EXPECT_NEAR(result, 5, 1000 * std::numeric_limits<double>::epsilon());
  }
}

// Tests whether quadrature is able to produce the accurate solution given at
// http://math2.uncc.edu/~shaodeng/TEACHING/math5172/Lectures/Lect_15.PDF p. 12.
GTEST_TEST(TriangleQuadrature, SecondOrder) {
  // Function to be integrated: (2 - x - 2y)
  auto f = [](const Vector2d& p) -> double {
    return 2 - p[0] - 2 * p[1];
  };

  // The triangle vertices.
  Vector2d v[3] = { {0.0, 0.0}, {1.0, 0.5}, {0.0, 1.0} };

  // Test Gaussian quadrature rules of orders 2 through 5.
  for (int order = 2; order <= 5; ++order) {
    GaussianTriangleQuadratureRule rule(order);

    // Compute the integral.
    double result = TriangleQuadrature<double, double>::Integrate(
        v[0], v[1], v[2], f, rule, 0.0);
    EXPECT_NEAR(result, 1.0/3, 1000 * std::numeric_limits<double>::epsilon());
  }
}

