#include "drake/geometry/proximity/bounding_volume_hierarchy.h"

namespace drake {
namespace geometry {
namespace internal {

bool Aabb::HasOverlap(const Aabb& a, const Aabb& b,
                      const math::RigidTransform<double>& X_AB) {
  // We need to split the transform into the position and rotation components,
  // `p_AB` and `R_AB`. For the purposes of streamlining the math below, they
  // will henceforth be named `t` and `r` respectively.
  const Vector3<double>& t = X_AB.translation() - a.center() + b.center();
  const Matrix3<double>& r = X_AB.rotation().matrix();

  // Compute some common subexpressions and add epsilon to counteract
  // arithmetic error, e.g. when two edges are parallel.
  Matrix3<double> abs_r = r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      abs_r(i, j) = abs(abs_r(i, j)) + std::numeric_limits<double>::epsilon();
    }
  }

  // First category of cases separating along a's axes.
  for (int i = 0; i < 3; ++i) {
    if (abs(t[i]) > a.half_width()[i] + b.half_width()[0] * abs_r(i, 0) +
                        b.half_width()[1] * abs_r(i, 1) +
                        b.half_width()[2] * abs_r(i, 2)) {
      return false;
    }
  }

  // Second category of cases separating along b's axes.
  for (int i = 0; i < 3; ++i) {
    if (abs(t[0] * r(0, i) + t[1] * r(1, i) + t[2] * r(2, i)) >
        b.half_width()[i] + a.half_width()[0] * abs_r(0, i) +
            a.half_width()[1] * abs_r(1, i) + a.half_width()[2] * abs_r(2, i)) {
      return false;
    }
  }

  // Third category of cases separating along the axes formed from the cross
  // products of a's and b's axes.
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (abs(t[(i + 2) % 3] * r((i + 1) % 3, j) -
              t[(i + 1) % 3] * r((i + 2) % 3, j)) >
          a.half_width()[(i + 1) % 3] * abs_r((i + 2) % 3, j) +
              a.half_width()[(i + 2) % 3] * abs_r((i + 1) % 3, j) +
              b.half_width()[(j + 1) % 3] * abs_r(i, (j + 2) % 3) +
              b.half_width()[(j + 2) % 3] * abs_r(i, (j + 1) % 3)) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
