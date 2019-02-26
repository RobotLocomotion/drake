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

// A class for tests from TEST04 of:
// NOLINTNEXTLINE(whitespace/line_length)
// https://people.sc.fsu.edu/~jburkardt/f_src/triangle_dunavant_rule/triangle_dunavant_rule_prb_output.txt
// where each result, integrated over the domain of the unit triangle, should
// equal unity (1.0).
class UnityQuadratureTest : public ::testing::Test {
 public:
  void TestForUnityResultFromStartingOrder(
      std::function<double(const Vector2d&> f, int starting_order) {
    // The vertices of the "unit triangle."
    TriangleVertices v = UnitTriangleVertices();

    // Test Gaussian quadrature rules of orders 1 through 5.
    for (int order = starting_order; order <= 5; ++order) {
      GaussianTriangleQuadratureRule rule(order);

      // Compute the integral.
      double result = TriangleQuadrature<double, double>::Integrate(
          v[0], v[1], v[2], f, rule, 0.0);
      EXPECT_NEAR(result, 1.0, 1000 * std::numeric_limits<double>::epsilon());
    }
  }

 private:
  TriangleVertices UnitTriangleVertices() {
    return {{ {0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0} }};
  }
}

// First test from TEST04.
TEST_F(UnityQuadratureTest, FirstOrder1) {
  // Function to be integrated: 2.0
  auto f = [](const Vector2d& p) -> double {
    return 2.0;
  };

  TestForUnityResultFromStartingOrder(f, 1);
}

// Second test from TEST04.
TEST_F(UnityQuadratureTest, FirstOrder2) {
  // Function to be integrated: 6.0y
  auto f = [](const Vector2d& p) -> double {
    return 6.0 * p[1];
  };

  TestForUnityResultFromStartingOrder(f, 1);
}

// Eleventh test from TEST04.
TEST_F(UnityQuadratureTest, FirstOrder3) {
  // Function to be integrated: 6x
  auto f = [](const Vector2d& p) -> double {
    return 6 * p[0];
  };

  TestForUnityResultFromStartingOrder(f, 1);
}

// Third test from TEST04 of:
TEST_F(UnityQuadratureTest, SecondOrder1) {
  // Function to be integrated: 12y²
  auto f = [](const Vector2d& p) -> double {
    return 12 * p[1] * p[1];
  };

  TestForUnityResultFromStartingOrder(f, 2);
}

// Twelfth test from TEST04 of:
TEST_F(UnityQuadratureTest, SecondOrder3) {
  // Function to be integrated: 24xy
  auto f = [](const Vector2d& p) -> double {
    return 24 * p[0] * p[1];
  };

  TestForUnityResultFromStartingOrder(f, 2);
}

// Fourth test from TEST04 of:
TEST_F(UnityQuadratureTest, ThirdOrder) {
  // Function to be integrated: 20y³
  auto f = [](const Vector2d& p) -> double {
    return 20 * p[1] * p[1] * p[1];
  };

  TestForUnityResultFromStartingOrder(f, 3);
}

// Thirteenth test from TEST04 of:
TEST_F(UnityQuadratureTest, ThirdOrder2) {
  // Function to be integrated: 60xy²
  auto f = [](const Vector2d& p) -> double {
    return 60 * p[0] * p[1] * p[1];
  };

  TestForUnityResultFromStartingOrder(f, 3);
}

// Fifth test from TEST04 of:
TEST_F(UnityQuadratureTest, FourthOrder1) {
  // Function to be integrated: 30y⁴
  auto f = [](const Vector2d& p) -> double {
    return 30 * p[1] * p[1] * p[1] * p[1];
  };

  TestForUnityResultFromStartingOrder(f, 4);
}

// Fourteenth test from TEST04 of:
TEST_F(UnityQuadratureTest, FourthOrder2) {
  // Function to be integrated: 120xy³
  auto f = [](const Vector2d& p) -> double {
    return 30 * p[1] * p[1] * p[1] * p[1];
  };

  TestForUnityResultFromStartingOrder(f, 4);
}

// Fifth test from TEST04 of:
TEST_F(UnityQuadratureTest, FifthOrder1) {
  // Function to be integrated: 30y⁵
  auto f = [](const Vector2d& p) -> double {
    return 42 * p[1] * p[1] * p[1] * p[1] * p[1];
  };

  TestForUnityResultFromStartingOrder(f, 5);
}

// Fifteenth test from TEST04 of:
TEST_F(UnityQuadratureTest, FifthOrder2) {
  // Function to be integrated: 210xy⁴
  auto f = [](const Vector2d& p) -> double {
    return 210 * p[0] * p[1] * p[1] * p[1] * p[1];
  };

  TestForUnityResultFromStartingOrder(f, 5);
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

