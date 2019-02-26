#include "drake/multibody/plant/hydroelastic_contact/triangle_quadrature.h"

#include <numeric>

#include <gtest/gtest.h>

#include "drake/multibody/plant/hydroelastic_contact/gaussian_triangle_quadrature_rule.h"

using Vector2d = drake::Vector2<double>;
using TriangleVertices = std::array<Vector2d, 3>;
using drake::multibody::hydroelastic_contact::TriangleQuadrature;
using drake::multibody::hydroelastic_contact::GaussianTriangleQuadratureRule;

GTEST_TEST(TriangleQuadrature, WeightsSumToUnity) {
  for (int order = 1; order <= 5; ++order) {
    GaussianTriangleQuadratureRule rule(order);
    const std::vector<double>& weights = rule.weights();
    const double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
    const double tol = 20 * weights.size() *
        std::numeric_limits<double>::epsilon();
    EXPECT_NEAR(sum, 1.0, tol);
  }
}

GTEST_TEST(TriangleQuadrature, BarycentricCoordsConsistent) {
  for (int order = 1; order <= 5; ++order) {
    GaussianTriangleQuadratureRule rule(order);
    const std::vector<Vector2d>& quadrature_points = rule.quadrature_points();
    for (const Vector2d& p : quadrature_points)
      EXPECT_LE(p[0] + p[1], 1.0 + std::numeric_limits<double>::epsilon());
  }
}

TriangleVertices UnitTriangleVertices() {
  return {{ {0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0} }};
}

// First test from TEST04 of:
// NOLINTNEXTLINE(whitespace/line_length)
// https://people.sc.fsu.edu/~jburkardt/f_src/triangle_dunavant_rule/triangle_dunavant_rule_prb_output.txt
GTEST_TEST(TriangleQuadrature, FirstOrder1) {
  // Function to be integrated: 2.0
  auto f = [](const Vector2d& p) -> double {
    return 2.0;
  };

  // The vertices of the "unit triangle."
  TriangleVertices v = UnitTriangleVertices();

  // Test Gaussian quadrature rules of orders 1 through 5.
  for (int order = 1; order <= 5; ++order) {
    GaussianTriangleQuadratureRule rule(order);

    // Compute the integral.
    double result = TriangleQuadrature<double, double>::Integrate(
        v[0], v[1], v[2], f, rule, 0.0);
    EXPECT_NEAR(result, 1.0, 1000 * std::numeric_limits<double>::epsilon());
  }
}

// Second test from TEST04 of:
// NOLINTNEXTLINE(whitespace/line_length)
// https://people.sc.fsu.edu/~jburkardt/f_src/triangle_dunavant_rule/triangle_dunavant_rule_prb_output.txt
GTEST_TEST(TriangleQuadrature, FirstOrder2) {
  // Function to be integrated: 6.0y
  auto f = [](const Vector2d& p) -> double {
    return 6.0 * p[1];
  };

  // The vertices of the "unit triangle."
  TriangleVertices v = UnitTriangleVertices();

  // Test Gaussian quadrature rules of orders 1 through 5.
  for (int order = 1; order <= 5; ++order) {
    GaussianTriangleQuadratureRule rule(order);

    // Compute the integral.
    double result = TriangleQuadrature<double, double>::Integrate(
        v[0], v[1], v[2], f, rule, 0.0);
    EXPECT_NEAR(result, 1.0, 1000 * std::numeric_limits<double>::epsilon());
  }
}

// Eleventh test from TEST04 of:
// NOLINTNEXTLINE(whitespace/line_length)
// https://people.sc.fsu.edu/~jburkardt/f_src/triangle_dunavant_rule/triangle_dunavant_rule_prb_output.txt
GTEST_TEST(TriangleQuadrature, FirstOrder3) {
  // Function to be integrated: 6x
  auto f = [](const Vector2d& p) -> double {
    return 6 * p[0];
  };

  // The vertices of the "unit triangle."
  TriangleVertices v = UnitTriangleVertices();

  // Test Gaussian quadrature rule of orders 1 through 5.
  for (int order = 1; order <= 5; ++order) {
    GaussianTriangleQuadratureRule rule(order);

    // Compute the integral.
    double result = TriangleQuadrature<double, double>::Integrate(
        v[0], v[1], v[2], f, rule, 0.0);
    EXPECT_NEAR(result, 1.0, 1000 * std::numeric_limits<double>::epsilon());
  }
}

// Third test from TEST04 of:
// NOLINTNEXTLINE(whitespace/line_length)
// https://people.sc.fsu.edu/~jburkardt/f_src/triangle_dunavant_rule/triangle_dunavant_rule_prb_output.txt
GTEST_TEST(TriangleQuadrature, SecondOrder1) {
  // Function to be integrated: 12y²
  auto f = [](const Vector2d& p) -> double {
    return 12 * p[1] * p[1];
  };

  // The vertices of the "unit triangle."
  TriangleVertices v = UnitTriangleVertices();

  // Test Gaussian quadrature rules of orders 2 through 5.
  for (int order = 2; order <= 5; ++order) {
    GaussianTriangleQuadratureRule rule(order);

    // Compute the integral.
    double result = TriangleQuadrature<double, double>::Integrate(
        v[0], v[1], v[2], f, rule, 0.0);
    EXPECT_NEAR(result, 1.0, 1000 * std::numeric_limits<double>::epsilon());
  }
}

// Tests whether quadrature is able to produce the accurate solution given at:
// http://math2.uncc.edu/~shaodeng/TEACHING/math5172/Lectures/Lect_15.PDF p. 12.
GTEST_TEST(TriangleQuadrature, SecondOrder2) {
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

// Twelfth test from TEST04 of:
// NOLINTNEXTLINE(whitespace/line_length)
// https://people.sc.fsu.edu/~jburkardt/f_src/triangle_dunavant_rule/triangle_dunavant_rule_prb_output.txt
GTEST_TEST(TriangleQuadrature, SecondOrder3) {
  // Function to be integrated: 24xy
  auto f = [](const Vector2d& p) -> double {
    return 24 * p[0] * p[1];
  };

  // The vertices of the "unit triangle."
  TriangleVertices v = UnitTriangleVertices();

  // Test Gaussian quadrature rule of orders 2 through 5.
  for (int order = 2; order <= 5; ++order) {
    GaussianTriangleQuadratureRule rule(order);

    // Compute the integral.
    double result = TriangleQuadrature<double, double>::Integrate(
        v[0], v[1], v[2], f, rule, 0.0);
    EXPECT_NEAR(result, 1.0, 1000 * std::numeric_limits<double>::epsilon());
  }
}

// Fourth test from TEST04 of:
// NOLINTNEXTLINE(whitespace/line_length)
// https://people.sc.fsu.edu/~jburkardt/f_src/triangle_dunavant_rule/triangle_dunavant_rule_prb_output.txt
GTEST_TEST(TriangleQuadrature, ThirdOrder) {
  // Function to be integrated: 20y³
  auto f = [](const Vector2d& p) -> double {
    return 20 * p[1] * p[1] * p[1];
  };

  // The vertices of the "unit triangle."
  TriangleVertices v = UnitTriangleVertices();

  // Test Gaussian quadrature rules of orders 3 through 5.
  for (int order = 3; order <= 5; ++order) {
    GaussianTriangleQuadratureRule rule(order);

    // Compute the integral.
    double result = TriangleQuadrature<double, double>::Integrate(
        v[0], v[1], v[2], f, rule, 0.0);
    EXPECT_NEAR(result, 1.0, 1000 * std::numeric_limits<double>::epsilon());
  }
}

// Thirteenth test from TEST04 of:
// NOLINTNEXTLINE(whitespace/line_length)
// https://people.sc.fsu.edu/~jburkardt/f_src/triangle_dunavant_rule/triangle_dunavant_rule_prb_output.txt
GTEST_TEST(TriangleQuadrature, ThirdOrder2) {
  // Function to be integrated: 60xy²
  auto f = [](const Vector2d& p) -> double {
    return 60 * p[0] * p[1] * p[1];
  };

  // The vertices of the "unit triangle."
  TriangleVertices v = UnitTriangleVertices();

  // Test Gaussian quadrature rule of orders 3 through 5.
  for (int order = 3; order <= 5; ++order) {
    GaussianTriangleQuadratureRule rule(order);

    // Compute the integral.
    double result = TriangleQuadrature<double, double>::Integrate(
        v[0], v[1], v[2], f, rule, 0.0);
    EXPECT_NEAR(result, 1.0, 1000 * std::numeric_limits<double>::epsilon());
  }
}

// Fifth test from TEST04 of:
// NOLINTNEXTLINE(whitespace/line_length)
// https://people.sc.fsu.edu/~jburkardt/f_src/triangle_dunavant_rule/triangle_dunavant_rule_prb_output.txt
GTEST_TEST(TriangleQuadrature, FourthOrder1) {
  // Function to be integrated: 30y⁴
  auto f = [](const Vector2d& p) -> double {
    return 30 * p[1] * p[1] * p[1] * p[1];
  };

  // The vertices of the "unit triangle."
  TriangleVertices v = UnitTriangleVertices();

  // Test Gaussian quadrature rules of orders 4 through 5.
  for (int order = 4; order <= 5; ++order) {
    GaussianTriangleQuadratureRule rule(order);

    // Compute the integral.
    double result = TriangleQuadrature<double, double>::Integrate(
        v[0], v[1], v[2], f, rule, 0.0);
    EXPECT_NEAR(result, 1.0, 1000 * std::numeric_limits<double>::epsilon());
  }
}

// Fourteenth test from TEST04 of:
// NOLINTNEXTLINE(whitespace/line_length)
// https://people.sc.fsu.edu/~jburkardt/f_src/triangle_dunavant_rule/triangle_dunavant_rule_prb_output.txt
GTEST_TEST(TriangleQuadrature, FourthOrder2) {
  // Function to be integrated: 120xy³
  auto f = [](const Vector2d& p) -> double {
    return 30 * p[1] * p[1] * p[1] * p[1];
  };

  // The vertices of the "unit triangle."
  TriangleVertices v = UnitTriangleVertices();

  // Test Gaussian quadrature rules of orders 4 through 5.
  for (int order = 4; order <= 5; ++order) {
    GaussianTriangleQuadratureRule rule(order);

    // Compute the integral.
    double result = TriangleQuadrature<double, double>::Integrate(
        v[0], v[1], v[2], f, rule, 0.0);
    EXPECT_NEAR(result, 1.0, 1000 * std::numeric_limits<double>::epsilon());
  }
}

// Fifth test from TEST04 of:
// NOLINTNEXTLINE(whitespace/line_length)
// https://people.sc.fsu.edu/~jburkardt/f_src/triangle_dunavant_rule/triangle_dunavant_rule_prb_output.txt
GTEST_TEST(TriangleQuadrature, FifthOrder1) {
  // Function to be integrated: 30y⁵
  auto f = [](const Vector2d& p) -> double {
    return 42 * p[1] * p[1] * p[1] * p[1] * p[1];
  };

  // The vertices of the "unit triangle."
  TriangleVertices v = UnitTriangleVertices();

  // Test Gaussian quadrature rule of order 5.
  const int order = 5;
  GaussianTriangleQuadratureRule rule(order);

  // Compute the integral.
  double result = TriangleQuadrature<double, double>::Integrate(
      v[0], v[1], v[2], f, rule, 0.0);
  EXPECT_NEAR(result, 1.0, 1000 * std::numeric_limits<double>::epsilon());
}

// Fifteenth test from TEST04 of:
// NOLINTNEXTLINE(whitespace/line_length)
// https://people.sc.fsu.edu/~jburkardt/f_src/triangle_dunavant_rule/triangle_dunavant_rule_prb_output.txt
GTEST_TEST(TriangleQuadrature, FifthOrder2) {
  // Function to be integrated: 210xy⁴
  auto f = [](const Vector2d& p) -> double {
    return 210 * p[0] * p[1] * p[1] * p[1] * p[1];
  };

  // The vertices of the "unit triangle."
  TriangleVertices v = UnitTriangleVertices();

  // Test Gaussian quadrature rule of order 5.
  const int order = 5;
  GaussianTriangleQuadratureRule rule(order);

  // Compute the integral.
  double result = TriangleQuadrature<double, double>::Integrate(
      v[0], v[1], v[2], f, rule, 0.0);
  EXPECT_NEAR(result, 1.0, 1000 * std::numeric_limits<double>::epsilon());
}

