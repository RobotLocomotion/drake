#pragma once

#include <array>

namespace drake {
namespace math {

/** Computes the real roots of the quadratic polynomial a⋅x² + b⋅x + c.

The roots are returned in ascending order, with NaN padding for missing
roots, and finite roots always come before the padding:
 - two real roots (including a repeated root): `{x₀, x₁}` with x₀ ≤ x₁;
 - one real root (i.e., when `a` is zero and the polynomial is linear):
   `{x₀, NaN}`;
 - no real roots: `{NaN, NaN}`. This includes the identically-zero
   polynomial, whose roots cannot be enumerated.

The coefficients may have any finite magnitudes; they are internally rescaled
so that intermediate computations neither overflow nor underflow. A root whose
magnitude exceeds the largest finite double is not reported (e.g., the large
root of a⋅x² + x - 1 for subnormal `a`), and is treated like any other missing
root.

@throws std::exception if any coefficient is not finite. */
std::array<double, 2> FindQuadraticRealRoots(double a, double b, double c);

}  // namespace math
}  // namespace drake
