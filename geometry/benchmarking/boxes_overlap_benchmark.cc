#include <benchmark/benchmark.h>
#include <fmt/format.h>

#include "drake/geometry/proximity/boxes_overlap.h"
#include "drake/geometry/test_utilities/boxes_overlap_transforms.h"

// These benchmarks help measure the cost of the BoxesOverlap query, used
// heavily in oriented bounding box (OBB) bounding volume hierarchy (BVH)
// calculations.

// Since the algorithm is commonly written with numerous early-exit branches,
// the benchmark leverages helper functions to construct relative poses to
// exercise all of the relevant cases. See also
// geometry/proximity/test/boxes_overlap_test.cc,
// geometry/test_utiilities/boxes_overlap_transform.{cc,h}.

// TODO(rpoyner-tri): consider adding a benchmark case with a schedule that
// defeats branch prediction, and maybe has some relationship to observed
// branch execution statistics.

namespace drake {
namespace geometry {
namespace internal {

// Parameters to construct specially posed test cases for the BoxesOverlap
// query. These will be used as input to the helper functions
// CalcEdgeTransform() or CalcCornerTransform(), to build a relative pose
// between boxes whose axis of minimum separation and query result are known
// ahead of time. The parameters use an implicit integer mapping for axes:
// (0=X, 1=Y, 2=Z).
struct PosedCaseParameters {
  // If true, construct a pose where the boxes overlap. Otherwise, construct a
  // pose without overlap.
  bool expect_overlap{};
  // The axis of object A along which the minimum distance should occur.
  int a_axis{};
  // If non-negative, call CalcEdgeTransform() instead of
  // CalcCornerTransform(). The non-negative value is the axis of object B used
  // to construct the cross-product axis (a_axis × b_axis) along which the
  // minimum distance should occur.
  int b_axis;
};

using Eigen::Vector3d;
using math::RigidTransformd;

class BoxesOverlapBenchmark : public benchmark::Fixture {
 public:
  // Calculate pose transform for posed cases, from benchmark args input.
  void SetupPosedCase(const benchmark::State& state) {
    const PosedCaseParameters acase{static_cast<bool>(state.range(0)),
                                    static_cast<int>(state.range(1)),
                                    static_cast<int>(state.range(2))};
    if (acase.b_axis >= 0) {
      X_AB = CalcEdgeTransform(a, b, acase.a_axis, acase.b_axis,
                               acase.expect_overlap);
    } else {
      X_AB = CalcCornerTransform(a, b, acase.a_axis, acase.expect_overlap);
    }
  }

  RigidTransformd X_AB;
  Vector3d a;
  Vector3d b;
};

// Directly encode a case where one box is entirely inside the other, and the
// axes are parallel.
BENCHMARK_DEFINE_F(BoxesOverlapBenchmark, ParallelContainedCase)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  a = Vector3d(1, 2, 1);
  b = Vector3d(0.5, 1, 0.5);
  X_AB = RigidTransformd{Vector3d{0.2, 0.4, 0.2}};
  const auto X_BA = X_AB.inverse();
  for (auto _ : state) {
    BoxesOverlap(a, b, X_AB);
    BoxesOverlap(b, a, X_BA);
  }
}

// Construct specially posed cases to ensure coverage of the many branches of
// the overlap algorithm.
BENCHMARK_DEFINE_F(BoxesOverlapBenchmark, PosedCase)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  a = Vector3d(2, 4, 3);
  b = Vector3d(3.5, 2, 1.5);
  SetupPosedCase(state);
  const auto X_BA = X_AB.inverse();
  for (auto _ : state) {
    BoxesOverlap(a, b, X_AB);
    BoxesOverlap(b, a, X_BA);
  }
}

BENCHMARK_REGISTER_F(BoxesOverlapBenchmark, ParallelContainedCase)
    ->Unit(benchmark::kNanosecond);

BENCHMARK_REGISTER_F(BoxesOverlapBenchmark, PosedCase)
    ->Unit(benchmark::kNanosecond)
    ->ArgsProduct({{false, true}, {0, 1, 2}, {-1, 0, 1, 2}});

}  // namespace internal
}  // namespace geometry
}  // namespace drake
