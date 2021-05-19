#include "drake/geometry/proximity/boxes_overlap.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using math::RigidTransformd;

// TODO(SeanCurtis-TRI) This code is tested in obb_test.cc for historical
//  reasons. If that causes confusion/difficulty, move it into its own unit test
//  and rework the Obb tests.
bool BoxesOverlap(const Vector3d& half_size_a, const Vector3d& half_size_b,
                  const RigidTransformd& X_AB) {
  // We need to split the transform into the position and rotation components,
  // `p_AB` and `R_AB`. For the purposes of streamlining the math below, they
  // will henceforth be named `t` and `r` respectively.
  const Vector3d& t = X_AB.translation();
  const Matrix3d& r = X_AB.rotation().matrix();

  // Compute some common subexpressions and add epsilon to counteract
  // arithmetic error, e.g. when two edges are parallel. We use the value as
  // specified from Gottschalk's OBB robustness tests.
  const double kEpsilon = 0.000001;
  Matrix3d abs_r = r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      abs_r(i, j) = abs(abs_r(i, j)) + kEpsilon;
    }
  }

  // First category of cases separating along a's axes.
  for (int i = 0; i < 3; ++i) {
    if (abs(t[i]) > half_size_a[i] + half_size_b.dot(abs_r.block<1, 3>(i, 0))) {
      return false;
    }
  }

  // Second category of cases separating along b's axes.
  for (int i = 0; i < 3; ++i) {
    if (abs(t.dot(r.block<3, 1>(0, i))) >
        half_size_b[i] + half_size_a.dot(abs_r.block<3, 1>(0, i))) {
      return false;
    }
  }

  // Third category of cases separating along the axes formed from the cross
  // products of a's and b's axes.
  int i1 = 1;
  for (int i = 0; i < 3; ++i) {
    const int i2 = (i1 + 1) % 3;  // Calculate common sub expressions.
    int j1 = 1;
    for (int j = 0; j < 3; ++j) {
      const int j2 = (j1 + 1) % 3;
      if (abs(t[i2] * r(i1, j) - t[i1] * r(i2, j)) >
          half_size_a[i1] * abs_r(i2, j) + half_size_a[i2] * abs_r(i1, j) +
              half_size_b[j1] * abs_r(i, j2) + half_size_b[j2] * abs_r(i, j1)) {
        return false;
      }
      j1 = j2;
    }
    i1 = i2;
  }

  return true;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
