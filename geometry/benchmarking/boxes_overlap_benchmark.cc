#include <benchmark/benchmark.h>
#include <fmt/format.h>

#include "drake/geometry/proximity/boxes_overlap.h"
#include "drake/geometry/test_utilities/boxes_overlap_transforms.h"

// These benchmarks help measure the cost of the BoxesOverlap query, used
// heavily in oriented bounding box (OBB) bounding volume hierarchy (BVH)
// calculations.

namespace drake {
namespace geometry {
namespace internal {

struct Case {
  bool expect_overlap{};
  int a_axis{};
  std::optional<int> b_axis;
};

using Eigen::Vector3d;
using math::RigidTransformd;

class BoxesOverlapBenchmark : public benchmark::Fixture {
 public:
  void SetupCase(const benchmark::State& state) {
    Case acase;
    acase.expect_overlap = state.range(0);
    acase.a_axis = state.range(1);
    int maybe_b_axis = state.range(2);
    if (maybe_b_axis >= 0) {
      acase.b_axis = maybe_b_axis;
    }
    if (acase.b_axis.has_value()) {
      X_AB = CalcEdgeTransform(a, b, acase.a_axis, *acase.b_axis,
                               acase.expect_overlap);
    } else {
      X_AB = CalcCornerTransform(a, b, acase.a_axis, acase.expect_overlap);
    }
  }

  RigidTransformd X_AB;
  Vector3d a;
  Vector3d b;
};

BENCHMARK_DEFINE_F(BoxesOverlapBenchmark, ParallelContainedCase)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  a = Vector3d(1, 2, 1);
  b = Vector3d(0.5, 1, 0.5);
  X_AB = RigidTransformd{Vector3d{0.2, 0.4, 0.2}};
  auto X_BA = X_AB.inverse();
  for (auto _ : state) {
    BoxesOverlap(a, b, X_AB);
    BoxesOverlap(b, a, X_BA);
  }
}

BENCHMARK_DEFINE_F(BoxesOverlapBenchmark, PosedCase)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  a = Vector3d(2, 4, 3);
  b = Vector3d(3.5, 2, 1.5);
  SetupCase(state);
  auto X_BA = X_AB.inverse();
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
