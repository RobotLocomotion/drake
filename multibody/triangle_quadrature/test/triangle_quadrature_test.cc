#include "drake/multibody/triangle_quadrature/triangle_quadrature.h"

#include <algorithm>
#include <numeric>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/triangle_quadrature/gaussian_triangle_quadrature_rule.h"

using Vector2d = drake::Vector2<double>;

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(TriangleQuadrature, GaussianQuadratureRuleThrowsAboveMaxOrder) {
  // TODO(edrumwri): Consider adding a max_order() method to the
  // TriangleQuadratureRule class (if more quadrature rules are added).
  const int max_order = 5;
  DRAKE_EXPECT_THROWS_MESSAGE(GaussianTriangleQuadratureRule(max_order + 1),
      std::logic_error, ".*quadrature only supported up to fifth order.*");
}

GTEST_TEST(TriangleQuadrature, WeightsSumToUnity) {
  for (int order = 1; order <= 5; ++order) {
    GaussianTriangleQuadratureRule rule(order);
    const std::vector<double>& weights = rule.weights();
    const double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
    const double tol = weights.size() *
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
      const std::function<double(const Vector2d&)>& f, int starting_order) {
    // Test Gaussian quadrature rules from starting_order through 5.
    for (int order = starting_order; order <= 5; ++order) {
      GaussianTriangleQuadratureRule rule(order);

      // Compute the integral over the unit triangle (0, 0), (1, 0), (0, 1).
      const int num_weights = static_cast<int>(rule.weights().size());
      double result = TriangleQuadrature<double, double>::Integrate(
          f, rule, 0.5 /* triangle area */);
      EXPECT_NEAR(result, 1.0,
          5 * num_weights * std::numeric_limits<double>::epsilon()) << order;
    }
  }
};

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
TEST_F(UnityQuadratureTest, SecondOrder2) {
  // Function to be integrated: 24xy
  auto f = [](const Vector2d& p) -> double {
    return 24 * p[0] * p[1];
  };

  TestForUnityResultFromStartingOrder(f, 2);
}

// Fourth test from TEST04 of:
TEST_F(UnityQuadratureTest, ThirdOrder1) {
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
    return 120 * p[0] * p[1] * p[1] * p[1];
  };

  TestForUnityResultFromStartingOrder(f, 4);
}

// Fifth test from TEST04 of:
TEST_F(UnityQuadratureTest, FifthOrder1) {
  // Function to be integrated: 42y⁵
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

}  // namespace
}  // namespace multibody
}  // namespace drake
