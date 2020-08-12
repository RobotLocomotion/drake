#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
class ComputePolynomialBasisUpToDegreeTest : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
  const Variable w_{"w"};

  void SetUp() override {
    EXPECT_PRED2(test::VarLess, x_, y_);
    EXPECT_PRED2(test::VarLess, y_, z_);
    EXPECT_PRED2(test::VarLess, z_, w_);
  }
};

TEST_F(ComputePolynomialBasisUpToDegreeTest, ChebyshevAnyParity) {
  // Test ComputePolynomialBasisUpToDegree with degree_type=DegreeType::kAny.
  // The chebyshev basis of x_ up to degree 0 is [1].
  const Vector1<ChebyshevBasisElement> result0 =
      ComputePolynomialBasisUpToDegree<1, ChebyshevBasisElement>(
          Variables({x_}), 0, internal::DegreeType::kAny);
  EXPECT_EQ(result0(0), ChebyshevBasisElement());

  // The chebyshev basis of x_ up to degree 1 is [1, T1(x)].
  const Vector2<ChebyshevBasisElement> result1 =
      ComputePolynomialBasisUpToDegree<2, ChebyshevBasisElement>(
          Variables({x_}), 1, internal::DegreeType::kAny);
  EXPECT_EQ(result1(0), ChebyshevBasisElement());
  EXPECT_EQ(result1(1), ChebyshevBasisElement(x_));

  // The chebyshev basis of x_ up to degree 2 is [1, T1(x), T2(x)].
  const Vector3<ChebyshevBasisElement> result2 =
      ComputePolynomialBasisUpToDegree<3, ChebyshevBasisElement>(
          Variables({x_}), 2, internal::DegreeType::kAny);
  EXPECT_EQ(result2(0), ChebyshevBasisElement());
  EXPECT_EQ(result2(1), ChebyshevBasisElement(x_, 1));
  EXPECT_EQ(result2(2), ChebyshevBasisElement(x_, 2));

  // The chebyshev basis of (x, y) up to degree 0 is [1]. Also instantitate the
  // function with Eigen::Dynamic.
  const VectorX<ChebyshevBasisElement> result3 =
      ComputePolynomialBasisUpToDegree<Eigen::Dynamic, ChebyshevBasisElement>(
          Variables({x_, y_}), 0, internal::DegreeType::kAny);
  EXPECT_EQ(result3.rows(), 1);
  EXPECT_EQ(result3(0), ChebyshevBasisElement());

  // The chebyshev basis of (x, y) up to degree 1 is [1, T1(x), T1(y)].
  const VectorX<ChebyshevBasisElement> result4 =
      ComputePolynomialBasisUpToDegree<Eigen::Dynamic, ChebyshevBasisElement>(
          Variables({x_, y_}), 1, internal::DegreeType::kAny);
  EXPECT_EQ(result4.rows(), 3);
  EXPECT_EQ(result4(0), ChebyshevBasisElement());
  EXPECT_EQ(result4(1), ChebyshevBasisElement(x_));
  EXPECT_EQ(result4(2), ChebyshevBasisElement(y_));

  // The chebyshev basis of (x, y) up to degree 2 is [1, T1(x), T1(y), T2(x),
  // T1(x)T1(y) T2(y)].
  const VectorX<ChebyshevBasisElement> result5 =
      ComputePolynomialBasisUpToDegree<Eigen::Dynamic, ChebyshevBasisElement>(
          Variables({x_, y_}), 2, internal::DegreeType::kAny);
  EXPECT_EQ(result5.rows(), 6);
  EXPECT_EQ(result5(0), ChebyshevBasisElement());
  EXPECT_EQ(result5(1), ChebyshevBasisElement(x_));
  EXPECT_EQ(result5(2), ChebyshevBasisElement(y_));
  EXPECT_EQ(result5(3), ChebyshevBasisElement(x_, 2));
  EXPECT_EQ(result5(4), ChebyshevBasisElement({{x_, 1}, {y_, 1}}));
  EXPECT_EQ(result5(5), ChebyshevBasisElement(y_, 2));

  // The chebyshev basis of (x, y) up to degree 3 is [1, T1(x), T1(y), T2(x),
  // T1(x)T1(y), T2(y), T3(x), T2(x)T1(y), T1(x)T2(y), T3(y)].
  const VectorX<ChebyshevBasisElement> result6 =
      ComputePolynomialBasisUpToDegree<Eigen::Dynamic, ChebyshevBasisElement>(
          Variables({x_, y_}), 3, internal::DegreeType::kAny);
  EXPECT_EQ(result6.rows(), 10);
  EXPECT_EQ(result6(0), ChebyshevBasisElement());
  EXPECT_EQ(result6(1), ChebyshevBasisElement(x_));
  EXPECT_EQ(result6(2), ChebyshevBasisElement(y_));
  EXPECT_EQ(result6(3), ChebyshevBasisElement(x_, 2));
  EXPECT_EQ(result6(4), ChebyshevBasisElement({{x_, 1}, {y_, 1}}));
  EXPECT_EQ(result6(5), ChebyshevBasisElement(y_, 2));
  EXPECT_EQ(result6(6), ChebyshevBasisElement(x_, 3));
  EXPECT_EQ(result6(7), ChebyshevBasisElement({{x_, 2}, {y_, 1}}));
  EXPECT_EQ(result6(8), ChebyshevBasisElement({{x_, 1}, {y_, 2}}));
  EXPECT_EQ(result6(9), ChebyshevBasisElement(y_, 3));

  // The chebyshev basis of (x, y, z) up to degree 2 is [1, T1(x), T1(y),
  // T1(z), T2(x), T1(x)T1(y), T1(x)T1(z), T2(y), T1(y)T1(z), T2(z)].
  const VectorX<ChebyshevBasisElement> result7 =
      ComputePolynomialBasisUpToDegree<Eigen::Dynamic, ChebyshevBasisElement>(
          Variables({x_, y_, z_}), 2, internal::DegreeType::kAny);
  EXPECT_EQ(result7.rows(), 10);
  EXPECT_EQ(result7(0), ChebyshevBasisElement());
  EXPECT_EQ(result7(1), ChebyshevBasisElement(x_));
  EXPECT_EQ(result7(2), ChebyshevBasisElement(y_));
  EXPECT_EQ(result7(3), ChebyshevBasisElement(z_));
  EXPECT_EQ(result7(4), ChebyshevBasisElement(x_, 2));
  EXPECT_EQ(result7(5), ChebyshevBasisElement({{x_, 1}, {y_, 1}}));
  EXPECT_EQ(result7(6), ChebyshevBasisElement({{x_, 1}, {z_, 1}}));
  EXPECT_EQ(result7(7), ChebyshevBasisElement(y_, 2));
  EXPECT_EQ(result7(8), ChebyshevBasisElement({{y_, 1}, {z_, 1}}));
  EXPECT_EQ(result7(9), ChebyshevBasisElement(z_, 2));

  // The chebyshev basis of (x, y, z) up to degree 3 is [1, T1(x), T1(y),
  // T1(z), T2(x) T1(x)T1(y), T1(x)T1(z), T2(y), T1(y)T1(z), T2(z), T3(x),
  // T2(x)T1(y), T2(x)T1(z), T1(x)T2(y), T1(x)T1(y)T1(z), T1(x)T2(z), T3(y),
  // T2(y)T1(z), T1(y)T2(z), T3(z)].
  const VectorX<ChebyshevBasisElement> result8 =
      ComputePolynomialBasisUpToDegree<Eigen::Dynamic, ChebyshevBasisElement>(
          Variables({x_, y_, z_}), 3, internal::DegreeType::kAny);
  EXPECT_EQ(result8.rows(), 20);
  EXPECT_EQ(result8(0), ChebyshevBasisElement());
  EXPECT_EQ(result8(1), ChebyshevBasisElement(x_));
  EXPECT_EQ(result8(2), ChebyshevBasisElement(y_));
  EXPECT_EQ(result8(3), ChebyshevBasisElement(z_));
  EXPECT_EQ(result8(4), ChebyshevBasisElement(x_, 2));
  EXPECT_EQ(result8(5), ChebyshevBasisElement({{x_, 1}, {y_, 1}}));
  EXPECT_EQ(result8(6), ChebyshevBasisElement({{x_, 1}, {z_, 1}}));
  EXPECT_EQ(result8(7), ChebyshevBasisElement(y_, 2));
  EXPECT_EQ(result8(8), ChebyshevBasisElement({{y_, 1}, {z_, 1}}));
  EXPECT_EQ(result8(9), ChebyshevBasisElement(z_, 2));
  EXPECT_EQ(result8(10), ChebyshevBasisElement(x_, 3));
  EXPECT_EQ(result8(11), ChebyshevBasisElement({{x_, 2}, {y_, 1}}));
  EXPECT_EQ(result8(12), ChebyshevBasisElement({{x_, 2}, {z_, 1}}));
  EXPECT_EQ(result8(13), ChebyshevBasisElement({{x_, 1}, {y_, 2}}));
  EXPECT_EQ(result8(14), ChebyshevBasisElement({{x_, 1}, {y_, 1}, {z_, 1}}));
  EXPECT_EQ(result8(15), ChebyshevBasisElement({{x_, 1}, {z_, 2}}));
  EXPECT_EQ(result8(16), ChebyshevBasisElement(y_, 3));
  EXPECT_EQ(result8(17), ChebyshevBasisElement({{y_, 2}, {z_, 1}}));
  EXPECT_EQ(result8(18), ChebyshevBasisElement({{y_, 1}, {z_, 2}}));
  EXPECT_EQ(result8(19), ChebyshevBasisElement(z_, 3));
}

TEST_F(ComputePolynomialBasisUpToDegreeTest, ChebyshevEvenParity) {
  // Test ComputePolynomialBasisUpToDegree with degree_type=DegreeType::kEven.

  // The chebyshev basis of x up to requested degree 0 or 1 (with even degrees)
  // is [1].
  for (int degree : {0, 1}) {
    const auto result =
        ComputePolynomialBasisUpToDegree<Eigen::Dynamic, ChebyshevBasisElement>(
            Variables({x_}), degree, internal::DegreeType::kEven);
    EXPECT_EQ(result.rows(), 1);
    EXPECT_EQ(result(0), ChebyshevBasisElement());
  }

  // The chebyshev basis of x up to requested degree 2 or 3 (with even degrees)
  // is [1, T2(x)].
  for (int degree : {2, 3}) {
    const auto result =
        ComputePolynomialBasisUpToDegree<Eigen::Dynamic, ChebyshevBasisElement>(
            Variables({x_}), degree, internal::DegreeType::kEven);
    EXPECT_EQ(result.rows(), 2);
    EXPECT_EQ(result(0), ChebyshevBasisElement());
    EXPECT_EQ(result(1), ChebyshevBasisElement(x_, 2));
  }

  // The chebyshev basis of (x, y) up to requested degree 2 or 3 (with even
  // degrees) is [1, T2(x), T1(x)T1(y) T2(y)].
  for (int degree : {2, 3}) {
    const auto result =
        ComputePolynomialBasisUpToDegree<Eigen::Dynamic, ChebyshevBasisElement>(
            Variables({x_, y_}), degree, internal::DegreeType::kEven);
    EXPECT_EQ(result.rows(), 4);
    EXPECT_EQ(result(0), ChebyshevBasisElement());
    EXPECT_EQ(result(1), ChebyshevBasisElement(x_, 2));
    EXPECT_EQ(result(2), ChebyshevBasisElement({{x_, 1}, {y_, 1}}));
    EXPECT_EQ(result(3), ChebyshevBasisElement(y_, 2));
  }
}

TEST_F(ComputePolynomialBasisUpToDegreeTest, ChebyshevOddParity) {
  // Test ComputePolynomialBasisUpToDegree with degree_type=DegreeType::kOdd.
  // The chebyshev basis of x up to degree 0 (with odd degrees) is [].
  const auto result1 =
      ComputePolynomialBasisUpToDegree<Eigen::Dynamic, ChebyshevBasisElement>(
          Variables({x_}), 0, internal::DegreeType::kOdd);
  EXPECT_EQ(result1.rows(), 0);

  // The chebyshev basis of x up to requested degree 1 or 2 (with odd degrees)
  // is [T1(x)].
  for (int degree : {1, 2}) {
    const auto result2 =
        ComputePolynomialBasisUpToDegree<Eigen::Dynamic, ChebyshevBasisElement>(
            Variables({x_}), degree, internal::DegreeType::kOdd);
    EXPECT_EQ(result2.rows(), 1);
    EXPECT_EQ(result2(0), ChebyshevBasisElement(x_));
  }

  // The chebyshev basis of x up to requestd degree 3 or 4 (with odd degrees) is
  // [T1(x), T3(x)].
  for (int degree : {3, 4}) {
    const auto result4 =
        ComputePolynomialBasisUpToDegree<Eigen::Dynamic, ChebyshevBasisElement>(
            Variables({x_}), degree, internal::DegreeType::kOdd);
    EXPECT_EQ(result4.rows(), 2);
    EXPECT_EQ(result4(0), ChebyshevBasisElement(x_));
    EXPECT_EQ(result4(1), ChebyshevBasisElement(x_, 3));
  }

  // The chebyshev basis of (x, y) up to requested degree 3 or 4 (with odd
  // degrees) is [T1(x), T1(y), T3(x), T2(x)T1(y), T1(x)T2(y), T3(y)].
  for (int degree : {3, 4}) {
    const auto result5 =
        ComputePolynomialBasisUpToDegree<Eigen::Dynamic, ChebyshevBasisElement>(
            Variables({x_, y_}), degree, internal::DegreeType::kOdd);
    EXPECT_EQ(result5.rows(), 6);
    EXPECT_EQ(result5(0), ChebyshevBasisElement(x_));
    EXPECT_EQ(result5(1), ChebyshevBasisElement(y_));
    EXPECT_EQ(result5(2), ChebyshevBasisElement(x_, 3));
    EXPECT_EQ(result5(3), ChebyshevBasisElement({{x_, 2}, {y_, 1}}));
    EXPECT_EQ(result5(4), ChebyshevBasisElement({{x_, 1}, {y_, 2}}));
    EXPECT_EQ(result5(5), ChebyshevBasisElement(y_, 3));
  }
}

TEST_F(ComputePolynomialBasisUpToDegreeTest, MonomialAnyParity) {
  // Test ComputePolynomialBasisUpToDegree with BasisElement = Monomial
  // The monomial basis of (x, y) up to requested degree 2 is [1, x, y, x^2, xy,
  // y^2]
  const auto result1 =
      ComputePolynomialBasisUpToDegree<Eigen::Dynamic, Monomial>(
          Variables({x_, y_}), 2, internal::DegreeType::kAny);
  EXPECT_EQ(result1.rows(), 6);
  EXPECT_EQ(result1(0), Monomial());
  EXPECT_EQ(result1(1), Monomial(x_));
  EXPECT_EQ(result1(2), Monomial(y_));
  EXPECT_EQ(result1(3), Monomial(x_, 2));
  EXPECT_EQ(result1(4), Monomial({{x_, 1}, {y_, 1}}));
  EXPECT_EQ(result1(5), Monomial(y_, 2));
}

}  // namespace symbolic
}  // namespace drake
