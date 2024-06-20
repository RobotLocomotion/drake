#include "drake/geometry/proximity/boxes_overlap.h"

#include <span>

#include <fmt/core.h>

namespace drake {
namespace geometry {
namespace internal {

namespace {
static constexpr int kAxisA0 = 0;
static constexpr int kAxisB0 = kAxisA0 + 3;
static constexpr int kAxisCross0 = kAxisB0 + 3;
static constexpr int kOverlap = kAxisCross0 + 9;
static constexpr int kEnd = kOverlap + 1;

static int64_t counters[kEnd] = {0};
}  // namespace

using Eigen::Matrix3d;
using Eigen::Vector3d;
using math::RigidTransformd;

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
  int stats_index = kAxisA0;
  for (int i = 0; i < 3; ++i) {
    if (abs(t[i]) > half_size_a[i] + half_size_b.dot(abs_r.block<1, 3>(i, 0))) {
      ++counters[stats_index];
      return false;
    }
    ++stats_index;
  }

  // Second category of cases separating along b's axes.
  DRAKE_ASSERT(stats_index == kAxisB0);
  for (int i = 0; i < 3; ++i) {
    if (abs(t.dot(r.block<3, 1>(0, i))) >
        half_size_b[i] + half_size_a.dot(abs_r.block<3, 1>(0, i))) {
      ++counters[stats_index];
      return false;
    }
    ++stats_index;
  }

  // Third category of cases separating along the axes formed from the cross
  // products of a's and b's axes.
  DRAKE_ASSERT(stats_index == kAxisCross0);
  int i1 = 1;
  for (int i = 0; i < 3; ++i) {
    const int i2 = (i1 + 1) % 3;  // Calculate common sub expressions.
    int j1 = 1;
    for (int j = 0; j < 3; ++j) {
      const int j2 = (j1 + 1) % 3;
      if (abs(t[i2] * r(i1, j) - t[i1] * r(i2, j)) >
          half_size_a[i1] * abs_r(i2, j) + half_size_a[i2] * abs_r(i1, j) +
              half_size_b[j1] * abs_r(i, j2) + half_size_b[j2] * abs_r(i, j1)) {
        ++counters[stats_index];
        return false;
      }
      j1 = j2;
      ++stats_index;
    }
    i1 = i2;
  }

  DRAKE_ASSERT(stats_index == kOverlap);
  ++counters[stats_index];
  return true;
}

void ReportStatistics() {
  std::span counts{counters};
  auto chunk = [&counts](int begin, int end) {
    return fmt::join(counts.subspan(begin, end - begin), ", ");
  };
  fmt::print(R"""(
A axis separation returns: [{}]
B axis separation returns: [{}]
Cross product axis separation returns [{}]
Overlap returns [{}]
)""",
             chunk(kAxisA0, kAxisB0), chunk(kAxisB0, kAxisCross0),
             chunk(kAxisCross0, kOverlap), chunk(kOverlap, kEnd));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
