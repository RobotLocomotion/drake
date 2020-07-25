#include <limits>
#include <ostream>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
const double kEps = std::numeric_limits<double>::epsilon();
GTEST_TEST(ChebyshevPolynomialTest, TestConstructor) {
  const symbolic::Variable x("x");
  const ChebyshevPolynomial p1(x, 0);
  EXPECT_EQ(p1.var(), x);
  EXPECT_EQ(p1.degree(), 0);
  const ChebyshevPolynomial p2(x, 1);
  EXPECT_EQ(p2.var(), x);
  EXPECT_EQ(p2.degree(), 1);
}

GTEST_TEST(ChebyshevPolynomialTest, ToPolynomial) {
  const symbolic::Variable x("x");
  const ChebyshevPolynomial p0(x, 0);
  EXPECT_PRED2(test::PolyEqual, p0.ToPolynomial(), symbolic::Polynomial(1));

  const ChebyshevPolynomial p1(x, 1);
  EXPECT_PRED2(test::PolyEqual, p1.ToPolynomial(), symbolic::Polynomial(x));

  const ChebyshevPolynomial p2(x, 2);
  EXPECT_PRED2(test::PolyEqual, p2.ToPolynomial(),
               symbolic::Polynomial(2 * x * x - 1));

  const ChebyshevPolynomial p3(x, 3);
  EXPECT_PRED2(test::PolyEqual, p3.ToPolynomial(),
               symbolic::Polynomial(4 * x * x * x - 3 * x));

  const ChebyshevPolynomial p4(x, 4);
  EXPECT_PRED2(test::PolyEqual, p4.ToPolynomial(),
               symbolic::Polynomial(8 * pow(x, 4) - 8 * x * x + 1));

  for (int n = 5; n < 20; ++n) {
    const ChebyshevPolynomial T_n_minus_2(x, n - 2);
    const ChebyshevPolynomial T_n_minus_1(x, n - 1);
    const ChebyshevPolynomial T_n(x, n);
    EXPECT_PRED2(
        test::PolyEqual, T_n.ToPolynomial(),
        symbolic::Polynomial(2 * x * T_n_minus_1.ToPolynomial().ToExpression() -
                             T_n_minus_2.ToPolynomial().ToExpression()));
  }
}

GTEST_TEST(ChebyshevPolynomialTest, Eval) {
  const symbolic::Variable x("x");
  const double tol = 10 * kEps;
  EXPECT_EQ(ChebyshevPolynomial(x, 0).Evaluate(2), 1);
  EXPECT_NEAR(ChebyshevPolynomial(x, 1).Evaluate(2), 2, tol);
  EXPECT_NEAR(ChebyshevPolynomial(x, 2).Evaluate(2), 7, tol);
  EXPECT_NEAR(ChebyshevPolynomial(x, 3).Evaluate(2), 26, tol);
  // Use the alternative definition for Chebyshev polynomial that
  // Tₙ(cos(θ)) = cos(nθ)
  const double theta = 0.2 * M_PI;
  const double cos_theta = std::cos(theta);
  for (int degree = 0; degree < 20; ++degree) {
    EXPECT_NEAR(ChebyshevPolynomial(x, degree).Evaluate(cos_theta),
                std::cos(degree * theta), tol);
  }
}

GTEST_TEST(ChebyshevPolynomialTest, equal_and_not_equal) {
  const symbolic::Variable x("x");
  EXPECT_TRUE(ChebyshevPolynomial(x, 1) == ChebyshevPolynomial(x, 1));
  EXPECT_EQ(ChebyshevPolynomial(x, 1), ChebyshevPolynomial(x, 1));
  EXPECT_TRUE(ChebyshevPolynomial(x, 2) == ChebyshevPolynomial(x, 2));
  EXPECT_EQ(ChebyshevPolynomial(x, 2), ChebyshevPolynomial(x, 2));
  EXPECT_TRUE(ChebyshevPolynomial(x, 3) == ChebyshevPolynomial(x, 3));
  EXPECT_EQ(ChebyshevPolynomial(x, 3), ChebyshevPolynomial(x, 3));
  const symbolic::Variable y("y");
  EXPECT_TRUE(ChebyshevPolynomial(x, 0) == ChebyshevPolynomial(y, 0));
  EXPECT_EQ(ChebyshevPolynomial(x, 0), ChebyshevPolynomial(y, 0));

  EXPECT_FALSE(ChebyshevPolynomial(x, 1) == ChebyshevPolynomial(x, 2));
  EXPECT_TRUE(ChebyshevPolynomial(x, 1) != ChebyshevPolynomial(x, 2));
  EXPECT_NE(ChebyshevPolynomial(x, 1), ChebyshevPolynomial(x, 2));
  EXPECT_FALSE(ChebyshevPolynomial(x, 1) == ChebyshevPolynomial(y, 1));
  EXPECT_TRUE(ChebyshevPolynomial(x, 1) != ChebyshevPolynomial(y, 1));
  EXPECT_NE(ChebyshevPolynomial(x, 1), ChebyshevPolynomial(y, 1));
}

GTEST_TEST(ChebyshevPolynomialTest, Differentiate) {
  const symbolic::Variable x("x");
  EXPECT_TRUE(ChebyshevPolynomial(x, 0).Differentiate().empty());

  const ChebyshevPolynomial p1(x, 1);
  // The gradient of T1(x) is 1 = T0(x)
  const std::vector<std::pair<ChebyshevPolynomial, double>> p1_derivative =
      p1.Differentiate();
  EXPECT_EQ(p1_derivative.size(), 1);
  EXPECT_EQ(p1_derivative[0].first, ChebyshevPolynomial(x, 0));
  EXPECT_EQ(p1_derivative[0].second, 1);

  const ChebyshevPolynomial p2(x, 2);
  // The gradient of T2(x) is 4*x = 4*T1(x)
  const std::vector<std::pair<ChebyshevPolynomial, double>> p2_derivative =
      p2.Differentiate();
  EXPECT_EQ(p2_derivative.size(), 1);
  EXPECT_EQ(p2_derivative[0].first, ChebyshevPolynomial(x, 1));
  EXPECT_EQ(p2_derivative[0].second, 4);

  const ChebyshevPolynomial p3(x, 3);
  // The gradient of T3(x) is 12*x^2 - 3 = 6*T2(x) + 3 * T0(x)
  const std::vector<std::pair<ChebyshevPolynomial, double>> p3_derivative =
      p3.Differentiate();
  EXPECT_EQ(p3_derivative.size(), 2);
  EXPECT_EQ(p3_derivative[0].first, ChebyshevPolynomial(x, 0));
  EXPECT_EQ(p3_derivative[0].second, 3);
  EXPECT_EQ(p3_derivative[1].first, ChebyshevPolynomial(x, 2));
  EXPECT_EQ(p3_derivative[1].second, 6);

  const ChebyshevPolynomial p4(x, 4);
  // The gradient of T4(x) is 32 * x^3 - 16 * x = 8*T3(x) + 8*T1(x)
  std::vector<std::pair<ChebyshevPolynomial, double>> p4_derivative =
      p4.Differentiate();
  EXPECT_EQ(p4_derivative.size(), 2);
  EXPECT_EQ(p4_derivative[0].first, ChebyshevPolynomial(x, 1));
  EXPECT_EQ(p4_derivative[0].second, 8);
  EXPECT_EQ(p4_derivative[1].first, ChebyshevPolynomial(x, 3));
  EXPECT_EQ(p4_derivative[1].second, 8);

  // Chebyshev polynomial has the property (1−x²)*dTₙ(x)/dx=n*(xTₙ(x)−Tₙ₊₁(x)) =
  // n*(Tₙ₋₁(x) − xTₙ(x)) for n >= 2. We check this property.
  for (int degree = 2; degree < 20; ++degree) {
    const ChebyshevPolynomial T_n(x, degree);
    const ChebyshevPolynomial T_n_minus_1(x, degree - 1);
    const ChebyshevPolynomial T_n_plus_1(x, degree + 1);
    const std::vector<std::pair<ChebyshevPolynomial, double>> T_derivative =
        T_n.Differentiate();
    if (degree % 2 == 0) {
      // Even degree Chebyshev polynomial
      // dTₙ(x)/dx = 2n ∑ⱼ Tⱼ(x), j is odd  and j <= n-1
      EXPECT_EQ(T_derivative.size(), degree / 2);
      for (int i = 0; i < degree / 2; ++i) {
        EXPECT_EQ(T_derivative[i].first.degree(), 2 * i + 1);
      }
    } else {
      // Odd degree Chebyshev polynomial
      // dTₙ(x)/dx = 2n ∑ⱼ Tⱼ(x) - n, j is even and j <= n-1
      EXPECT_EQ(T_derivative.size(), (degree + 1) / 2);
      for (int i = 0; i < (degree + 1) / 2; ++i) {
        EXPECT_EQ(T_derivative[i].first.degree(), 2 * i);
      }
    }
    symbolic::Polynomial T_derivative_poly(0);
    for (const auto& pair : T_derivative) {
      T_derivative_poly += pair.first.ToPolynomial() * pair.second;
    }
    // lhs is (1−x²)*dTₙ(x)/dx
    const symbolic::Polynomial lhs =
        Polynomial(1 - x * x, {x}) * T_derivative_poly;
    EXPECT_PRED2(test::PolyEqual, lhs,
                 degree * (x * T_n.ToPolynomial() - T_n_plus_1.ToPolynomial()));
    EXPECT_PRED2(
        test::PolyEqual, lhs,
        degree * (T_n_minus_1.ToPolynomial() - x * T_n.ToPolynomial()));
  }
}

GTEST_TEST(ChebyshevPolynomialTest, UnorderedMapOfChebyshevPolynomial) {
  // This test shows that we can use a std::unordered_map whose key is of
  // ChebyshevPolynomial, namely we can hash ChebyshevPolynomial.
  const symbolic::Variable x("x");
  std::unordered_map<ChebyshevPolynomial, int> chebyshev_to_coeff_map;
  chebyshev_to_coeff_map.emplace(ChebyshevPolynomial(x, 0), 1);
  chebyshev_to_coeff_map.emplace(ChebyshevPolynomial(x, 1), 2);
  chebyshev_to_coeff_map.emplace(ChebyshevPolynomial(x, 2), 3);

  const auto it1 = chebyshev_to_coeff_map.find(ChebyshevPolynomial(x, 1));
  ASSERT_TRUE(it1 != chebyshev_to_coeff_map.end());
  EXPECT_EQ(it1->second, 2);

  EXPECT_EQ(chebyshev_to_coeff_map.find(ChebyshevPolynomial(x, 3)),
            chebyshev_to_coeff_map.end());
  // T0(y) should also be found in the map, since T0(x) is a key of the map, and
  // T0(x) = T0(y) = 1.
  const symbolic::Variable y("y");
  const auto it0 = chebyshev_to_coeff_map.find(ChebyshevPolynomial(y, 0));
  ASSERT_NE(it0, chebyshev_to_coeff_map.end());
  EXPECT_EQ(it0->second, 1);
}

GTEST_TEST(ChebyshevPolynomial, operator_out) {
  const Variable x("x");
  std::ostringstream os1;
  os1 << ChebyshevPolynomial(x, 0);
  EXPECT_EQ(fmt::format("{}", os1.str()), "T0()");

  std::ostringstream os2;
  os2 << ChebyshevPolynomial(x, 2);
  EXPECT_EQ(fmt::format("{}", os2.str()), "T2(x)");
}

GTEST_TEST(ChebyshevPolynomialTest, CompareChebyshevPolynomial) {
  const symbolic::Variable x("x");
  const symbolic::Variable y("y");
  ASSERT_TRUE(x.get_id() < y.get_id());

  EXPECT_FALSE(CompareChebyshevPolynomial()(ChebyshevPolynomial(x, 0),
                                            ChebyshevPolynomial(x, 0)));
  EXPECT_FALSE(CompareChebyshevPolynomial()(ChebyshevPolynomial(x, 0),
                                            ChebyshevPolynomial(y, 0)));
  EXPECT_TRUE(CompareChebyshevPolynomial()(ChebyshevPolynomial(x, 0),
                                           ChebyshevPolynomial(y, 1)));
  EXPECT_TRUE(CompareChebyshevPolynomial()(ChebyshevPolynomial(x, 0),
                                           ChebyshevPolynomial(x, 1)));
  EXPECT_FALSE(CompareChebyshevPolynomial()(ChebyshevPolynomial(x, 1),
                                            ChebyshevPolynomial(y, 0)));
  EXPECT_TRUE(CompareChebyshevPolynomial()(ChebyshevPolynomial(x, 1),
                                           ChebyshevPolynomial(y, 1)));
  EXPECT_TRUE(CompareChebyshevPolynomial()(ChebyshevPolynomial(x, 2),
                                           ChebyshevPolynomial(y, 1)));
  EXPECT_TRUE(CompareChebyshevPolynomial()(ChebyshevPolynomial(x, 2),
                                           ChebyshevPolynomial(y, 3)));
  EXPECT_FALSE(CompareChebyshevPolynomial()(ChebyshevPolynomial(y, 2),
                                            ChebyshevPolynomial(y, 1)));
}
}  // namespace symbolic
}  // namespace drake
