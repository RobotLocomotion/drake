#include "drake/math/real_roots.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace math {

namespace {

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

/* Rescales the coefficients by an exact power of two so that the largest
 magnitude lies in [1, 2). This changes neither the roots nor (barring
 underflow of negligible coefficients) any coefficient's significand.
 @pre At least one coefficient is non-zero. */
template <std::size_t N>
void NormalizeByPowerOfTwo(std::array<double, N>* coeffs) {
  double m = 0;
  for (const double c : *coeffs) m = std::max(m, std::abs(c));
  DRAKE_ASSERT(m > 0);
  const int e = std::ilogb(m);
  if (e >= std::numeric_limits<double>::min_exponent - 1) {
    // Multiplying by a (representable) power of two is exact, barring
    // underflow.
    const double scale = std::scalbn(1.0, -e);
    for (double& c : *coeffs) c *= scale;
  } else {
    // The largest coefficient is subnormal, so 2⁻ᵉ is not representable.
    for (double& c : *coeffs) c = std::scalbn(c, -e);
  }
}

/* Returns the finite values among x0 and x1 in ascending order, with NaN
 padding at the tail. Non-finite values are roots whose magnitude exceeds the
 largest finite double. */
std::array<double, 2> SortFiniteRoots(double x0, double x1) {
  using std::isfinite;
  if (!isfinite(x0)) std::swap(x0, x1);
  if (!isfinite(x0)) return {NaN, NaN};
  if (!isfinite(x1)) return {x0, NaN};
  if (x1 < x0) std::swap(x0, x1);
  return {x0, x1};
}

/* Solves for the real roots of a⋅x² + b⋅x + c with coefficients that are
 already scaled so that the discriminant b² - 4ac cannot overflow, and so that
 where one of its terms underflows, that term is negligible relative to the
 other. FindQuadraticRealRootsImpl() establishes this. A root itself may still
 overflow (e.g., -c/b with tiny b); such roots are not reported.
 @pre The coefficients are finite, not all zero, and scaled as described. */
std::array<double, 2> FindScaledQuadraticRealRoots(double a, double b,
                                                   double c) {
  using std::abs;
  using std::copysign;
  using std::sqrt;

  // Handle degenerate cases.
  if (a == 0) {
    // Non-zero constant polynomial has no roots.
    if (b == 0) {
      return {NaN, NaN};
    }
    // Linear case.
    return SortFiniteRoots(-c / b, NaN);
  }
  if (b == 0) {
    // Double root at x = 0.
    if (c == 0) {
      return {0.0, 0.0};
    }
    // No real roots.
    if (std::signbit(a) == std::signbit(c)) {
      return {NaN, NaN};
    }
    // The ratio -c/a can overflow or underflow even when its square root is
    // comfortably representable; then take the ratio of square roots instead.
    const double x0_sq = -c / a;
    const double x0 =
        std::isnormal(x0_sq) ? sqrt(x0_sq) : sqrt(abs(c)) / sqrt(abs(a));
    return {-x0, x0};
  }
  if (c == 0) {
    return SortFiniteRoots(-b / a, 0.0);
  }

  // General case.
  const double discriminant = b * b - 4.0 * a * c;

  // No real roots.
  if (discriminant < 0) {
    return {NaN, NaN};
  }
  // Avoid catastrophic cancellation. With |q| ≥ |b|/2, c/q cannot overflow:
  // a real root requires either b² ≥ 4ac or ac < 0, and in the latter case
  // |q| ≥ √|ac|, so |c/q| ≤ √|c/a|.
  const double q = -0.5 * (b + copysign(sqrt(discriminant), b));
  const double x0 = q / a;
  const double x1 = c / q;
  if (!std::isfinite(x0)) return {x1, NaN};
  if (x0 < x1) return {x0, x1};
  return {x1, x0};
}

/* Solves for the real roots of a⋅x² + b⋅x + c with arbitrary finite
 coefficients (see FindQuadraticRealRoots()), by scaling them for
 FindScaledQuadraticRealRoots().

 When every non-zero coefficient's magnitude lies in [2⁻⁵⁰⁰, 2⁵⁰⁰], they need
 no scaling: b² and 4ac can neither overflow nor underflow. Otherwise, the
 coefficients are normalized by a power of two (see NormalizeByPowerOfTwo()).
 Normalizing the coefficients alone is not enough: the ratio between them can
 be too extreme to survive normalization (e.g., 1e-170⋅x² - 1e170, whose roots
 ±1e170 are representable but whose a/c = 1e-340 is not). So, when `a` and `c`
 are both non-zero, x is also rescaled as x = 2ᵏ⋅y, with 2ᵏ ≈ √|c/a|, which
 makes the coefficients of y² and y⁰ comparable. If b² exceeds 4ac by so much
 that even this would underflow them, the roots are -b/a and -c/b to full
 double precision. */
std::array<double, 2> FindQuadraticRealRootsImpl(double a, double b, double c) {
  using std::ilogb;
  using std::scalbn;

  // Identically-zero polynomial: technically infinite roots, but we'll treat
  // this as no roots.
  if (a == 0 && b == 0 && c == 0) {
    return {NaN, NaN};
  }

  // The common case: no scaling needed. (A zero coefficient is fine too.)
  constexpr double kSafeMin = 0x1p-500;
  constexpr double kSafeMax = 0x1p500;
  const auto in_safe_range = [](double v) {
    const double m = std::abs(v);
    return v == 0 || (m >= kSafeMin && m <= kSafeMax);
  };
  if (in_safe_range(a) && in_safe_range(b) && in_safe_range(c)) {
    return FindScaledQuadraticRealRoots(a, b, c);
  }

  /* With a or c zero, the roots are {-c/b} or {0, -b/a}. Normalization can
   only underflow a coefficient that is so small relative to the others that
   the root depending on it isn't representable anyway. */
  if (a == 0 || c == 0) {
    std::array<double, 3> coeffs{a, b, c};
    NormalizeByPowerOfTwo(&coeffs);
    return FindScaledQuadraticRealRoots(coeffs[0], coeffs[1], coeffs[2]);
  }

  const int e_a = ilogb(a);
  const int e_c = ilogb(c);
  /* b² exceeds |4ac| by more than 2^kMaxGap. Then the discriminant is b² to
   full precision and the roots are -b/a and -c/b. This also bounds how small
   the balanced, normalized a and c can get (≈ 2^(-kMaxGap/2)), keeping them
   normal. */
  constexpr int kMaxGap = 2000;
  if (b != 0 && 2 * ilogb(b) - e_a - e_c > kMaxGap) {
    return SortFiniteRoots(-b / a, -c / b);
  }

  // Balance: the coefficients of y = x / 2ᵏ are a⋅2²ᵏ, b⋅2ᵏ, and c, which we
  // also normalize, in one exact scaling per coefficient.
  const int k = (e_c - e_a) / 2;
  int m = std::max(e_a + 2 * k, e_c);
  if (b != 0) m = std::max(m, ilogb(b) + k);
  const auto [y0, y1] = FindScaledQuadraticRealRoots(
      scalbn(a, 2 * k - m), scalbn(b, k - m), scalbn(c, -m));
  // Scaling by a positive factor preserves order; overflowed roots drop out.
  return SortFiniteRoots(scalbn(y0, k), scalbn(y1, k));
}

}  // namespace

std::array<double, 2> FindQuadraticRealRoots(double a, double b, double c) {
  DRAKE_THROW_UNLESS(std::isfinite(a));
  DRAKE_THROW_UNLESS(std::isfinite(b));
  DRAKE_THROW_UNLESS(std::isfinite(c));
  return FindQuadraticRealRootsImpl(a, b, c);
}

}  // namespace math
}  // namespace drake
