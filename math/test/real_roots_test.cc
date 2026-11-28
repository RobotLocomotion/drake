#include "drake/math/real_roots.h"

#include <cmath>
#include <limits>

#include <fmt/format.h>
#include <gtest/gtest.h>

namespace drake {
namespace math {
namespace {

using std::isfinite;
using std::isnan;

::testing::AssertionResult AreBothNaN(const std::array<double, 2>& roots) {
  if (isnan(roots[0]) && isnan(roots[1])) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "Expected both NaN, got [" << roots[0] << ", " << roots[1] << "]";
}

constexpr double kTol = std::numeric_limits<double>::epsilon();

GTEST_TEST(FindQuadraticRealRootsTest, DegenerateConstantNonZero) {
  // a = 0, b = 0, c != 0 -> no roots -> both NaN.
  const std::array<double, 2> roots = FindQuadraticRealRoots(0.0, 0.0, 1.0);
  EXPECT_TRUE(AreBothNaN(roots));
}

GTEST_TEST(FindQuadraticRealRootsTest, DegenerateConstantZero) {
  // a = 0, b = 0, c = 0 -> identically zero polynomial. Every point is a
  // root; the implementation cannot enumerate them and returns all NaN.
  const std::array<double, 2> roots = FindQuadraticRealRoots(0.0, 0.0, 0.0);
  EXPECT_TRUE(AreBothNaN(roots));
}

GTEST_TEST(FindQuadraticRealRootsTest, LinearCase) {
  // a = 0, b != 0 -> linear root at -c / b, second is NaN.
  const std::array<double, 2> roots = FindQuadraticRealRoots(0.0, 2.0, -6.0);
  ASSERT_TRUE(isfinite(roots[0]));
  EXPECT_EQ(roots[0], 6.0 / 2.0);
  EXPECT_TRUE(isnan(roots[1]));
}

GTEST_TEST(FindQuadraticRealRootsTest, DoubleRootAtZero) {
  // b = 0, c = 0 -> double root at 0.
  const std::array<double, 2> roots = FindQuadraticRealRoots(1.0, 0.0, 0.0);
  EXPECT_EQ(roots[0], 0.0);
  EXPECT_EQ(roots[1], 0.0);
}

GTEST_TEST(FindQuadraticRealRootsTest, NoRealRootsFromBSymmetric) {
  // b = 0, a*c > 0 -> no real roots.
  const std::array<double, 2> roots = FindQuadraticRealRoots(1.0, 0.0, 1.0);
  EXPECT_TRUE(AreBothNaN(roots));
}

GTEST_TEST(FindQuadraticRealRootsTest, SymmetricNonZeroRootsFromBSymmetric) {
  // b = 0, a*c < 0 -> roots are ±sqrt(-c/a), ordered ascending.
  const std::array<double, 2> roots = FindQuadraticRealRoots(1.0, 0.0, -4.0);
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  EXPECT_NEAR(roots[0], -2.0, kTol);
  EXPECT_NEAR(roots[1], 2.0, kTol);
}

GTEST_TEST(FindQuadraticRealRootsTest, CZeroNegativeFirst) {
  // c = 0 and -b/a < 0 -> {x0, 0}.
  const std::array<double, 2> roots = FindQuadraticRealRoots(1.0, 2.0, 0.0);
  // Polynomial x^2 + 2x = x(x+2) -> roots -2 and 0.
  EXPECT_EQ(roots[0], -2.0);
  EXPECT_EQ(roots[1], 0.0);
}

GTEST_TEST(FindQuadraticRealRootsTest, CZeroPositiveFirst) {
  // c = 0 and -b/a >= 0 -> {0, x0}.
  const std::array<double, 2> roots = FindQuadraticRealRoots(-1.0, 2.0, 0.0);
  // Polynomial -x^2 + 2x = -x(x-2) -> roots 0 and 2.
  EXPECT_EQ(roots[0], 0.0);
  EXPECT_EQ(roots[1], 2.0);
}

GTEST_TEST(FindQuadraticRealRootsTest, GeneralNoRealRoots) {
  // Discriminant < 0.
  const std::array<double, 2> roots = FindQuadraticRealRoots(1.0, 2.0, 5.0);
  EXPECT_TRUE(AreBothNaN(roots));
}

GTEST_TEST(FindQuadraticRealRootsTest, GeneralTwoRealRootsAscendingOrder) {
  //  x^2 - 5x + 6 = 0 -> roots 2 and 3.
  const std::array<double, 2> roots = FindQuadraticRealRoots(1.0, -5.0, 6.0);
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  EXPECT_NEAR(roots[0], 2.0, kTol);
  EXPECT_NEAR(roots[1], 3.0, kTol);
}

GTEST_TEST(FindQuadraticRealRootsTest, GeneralTwoRealRootsAlreadyInOrder) {
  // b > 0 branch: x^2 + x - 6 = 0 -> roots -3 and 2.
  const std::array<double, 2> roots = FindQuadraticRealRoots(1.0, 1.0, -6.0);
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  EXPECT_NEAR(roots[0], -3.0, kTol);
  EXPECT_NEAR(roots[1], 2.0, kTol);
}

GTEST_TEST(FindQuadraticRealRootsTest, ScaledCoefficients) {
  /* Scaling every coefficient by the same factor doesn't change the roots,
   even when the factor is so large or small that the discriminant b² - 4ac
   would overflow or underflow if computed directly. Scaling by a power of two
   keeps the coefficients exact. */
  for (const int e : {-1072, -1000, -600, -100, 0, 100, 600, 1000, 1021}) {
    SCOPED_TRACE(fmt::format("s = 2^{}", e));
    const double s = std::ldexp(1.0, e);
    // s⋅(x - 1)(x - 3).
    const std::array<double, 2> roots =
        FindQuadraticRealRoots(s, -4 * s, 3 * s);
    EXPECT_EQ(roots[0], 1.0);
    EXPECT_EQ(roots[1], 3.0);
    // s⋅(x² + x + 1) has no real roots.
    EXPECT_TRUE(AreBothNaN(FindQuadraticRealRoots(s, s, s)));
  }

  // Non-power-of-two scale factors.
  EXPECT_TRUE(AreBothNaN(FindQuadraticRealRoots(1e-200, 1e-200, 1e-200)));
  const std::array<double, 2> roots =
      FindQuadraticRealRoots(1e200, 2e200, 0.5e200);  // 1e200⋅(x² + 2x + ½).
  EXPECT_NEAR(roots[0], -1 - std::sqrt(0.5), 4 * kTol);
  EXPECT_NEAR(roots[1], -1 + std::sqrt(0.5), 4 * kTol);
}

GTEST_TEST(FindQuadraticRealRootsTest, RootsOfExtremeMagnitude) {
  /* With b = 0, the roots are ±√(-c/a). The ratio -c/a can overflow or
   underflow even though its square root is representable. */
  // x² - 2⁻¹⁰⁶⁰ = 0 (subnormal c) → x = ±2⁻⁵³⁰.
  std::array<double, 2> roots = FindQuadraticRealRoots(1, 0, -0x1p-1060);
  EXPECT_EQ(roots[0], -0x1p-530);
  EXPECT_EQ(roots[1], 0x1p-530);
  // 2⁻¹⁰⁶⁰⋅x² - 1 = 0 (subnormal a) → x = ±2⁵³⁰.
  roots = FindQuadraticRealRoots(0x1p-1060, 0, -1);
  EXPECT_EQ(roots[0], -0x1p530);
  EXPECT_EQ(roots[1], 0x1p530);
  // 1e-170⋅x² - 1e170 = 0 → x = ±1e170.
  roots = FindQuadraticRealRoots(1e-170, 0, -1e170);
  EXPECT_NEAR(roots[0], -1e170, 4 * kTol * 1e170);
  EXPECT_NEAR(roots[1], 1e170, 4 * kTol * 1e170);
}

GTEST_TEST(FindQuadraticRealRootsTest, CoefficientsOfWidelyDifferentMagnitude) {
  /* The ratio between coefficients can be too extreme to survive scaling the
   coefficients alone (which would underflow c to zero here). The roots are
   2ᵉ and (1 + 2⁻²⁰)⋅2ᵉ with e = -800; with a = 2⁶⁰⁰ every coefficient is an
   exact normal double: a = 2⁶⁰⁰, b ≈ -2⁻¹⁹⁹, c ≈ 2⁻¹⁰⁰⁰. */
  const double a = 0x1p600;
  const double r0 = 0x1p-800;
  const double r1 = (1 + 0x1p-20) * 0x1p-800;
  const std::array<double, 2> roots =
      FindQuadraticRealRoots(a, -a * (r0 + r1), a * r0 * r1);
  EXPECT_NEAR(roots[0], r0, 4 * kTol * r0);
  EXPECT_NEAR(roots[1], r1, 4 * kTol * r1);

  /* When b² exceeds 4ac by more than ~2²⁰⁰⁰, even rescaling x can't keep both
   a and c representable alongside b. Then the roots are -c/b and -b/a to full
   precision: 2⁻¹⁰⁷⁰⋅x² + 2⁵¹⁰⋅x + 2⁻⁷⁰ has roots ≈ -2⁻⁵⁸⁰ and ≈ -2¹⁵⁸⁰, and
   only the former is representable. */
  const std::array<double, 2> gap_roots =
      FindQuadraticRealRoots(0x1p-1070, 0x1p510, 0x1p-70);
  EXPECT_EQ(gap_roots[0], -0x1p-580);
  EXPECT_TRUE(isnan(gap_roots[1]));

  // Subnormal coefficients with c = 0: 2⁻¹⁰⁷²⋅(x² - 3x) → {0, 3}.
  const std::array<double, 2> zero_c_roots =
      FindQuadraticRealRoots(0x1p-1072, -3 * 0x1p-1072, 0);
  EXPECT_EQ(zero_c_roots[0], 0.0);
  EXPECT_EQ(zero_c_roots[1], 3.0);
}

GTEST_TEST(FindQuadraticRealRootsTest, RootsBeyondDoubleRangeAreNotReported) {
  // 2⁻¹⁰⁷⁰⋅x + 1 = 0 → x = -2¹⁰⁷⁰, which is not a finite double.
  EXPECT_TRUE(AreBothNaN(FindQuadraticRealRoots(0, 0x1p-1070, 1)));
  // 2⁻¹⁰⁷⁰⋅x² + x - 1 = 0 → x ≈ 1 and x ≈ -2¹⁰⁷⁰.
  std::array<double, 2> roots = FindQuadraticRealRoots(0x1p-1070, 1, -1);
  EXPECT_EQ(roots[0], 1.0);
  EXPECT_TRUE(isnan(roots[1]));
  // 2⁻¹⁰⁷⁰⋅x² + x = 0 → x = 0 and x = -2¹⁰⁷⁰.
  roots = FindQuadraticRealRoots(0x1p-1070, 1, 0);
  EXPECT_EQ(roots[0], 0.0);
  EXPECT_TRUE(isnan(roots[1]));
}

GTEST_TEST(FindQuadraticRealRootsTest, NonFiniteCoefficientsThrows) {
  const double inf = std::numeric_limits<double>::infinity();
  const double NaN = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(FindQuadraticRealRoots(inf, 1.0, 1.0),
               std::runtime_error);  // non-finite a
  EXPECT_THROW(FindQuadraticRealRoots(1.0, inf, 1.0),
               std::runtime_error);  // non-finite b
  EXPECT_THROW(FindQuadraticRealRoots(1.0, 1.0, inf),
               std::runtime_error);  // non-finite c
  EXPECT_THROW(FindQuadraticRealRoots(NaN, 1.0, 1.0),
               std::runtime_error);  // non-finite a
  EXPECT_THROW(FindQuadraticRealRoots(1.0, NaN, 1.0),
               std::runtime_error);  // non-finite b
  EXPECT_THROW(FindQuadraticRealRoots(1.0, 1.0, NaN),
               std::runtime_error);  // non-finite c
}

}  // namespace
}  // namespace math
}  // namespace drake
