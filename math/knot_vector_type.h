#pragma once

namespace drake {
namespace math {

/**
Enum representing types of knot vectors. "Uniform" refers to the spacing
between the knots. "Clamped" indicates that the first and last knots have
multiplicity equal to the order of the spline.

Reference:
http://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node17.html
*/
enum class KnotVectorType { kUniform, kClampedUniform };

}  // namespace math
}  // namespace drake
