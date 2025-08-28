#include <memory>
#include <tuple>
#include <utility>

#include <benchmark/benchmark.h>
#include <fmt/format.h>

#include "drake/geometry/proximity/hydroelastic_calculator.h"
#include "drake/geometry/proximity/hydroelastic_internal.h"
#include "drake/geometry/proximity/make_ellipsoid_field.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
#include "drake/geometry/proximity/make_sphere_field.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

// clang-format off
/* @defgroup compliant_mesh_intersection_benchmarks Compliant Mesh Intersection Benchmarks  // NOLINT
 @ingroup proximity_queries

 The benchmark evaluates mesh intersection between compliant meshes.

 It computes the contact surface formed from the intersection of an ellipsoid
 and a sphere using broad-phase culling (via a bounding volume hierarchy).
 Arguments include:
 - __resolution__
 - __contact overlap__

 For more details on arguments, see <b>Interpreting the benchmark</b> below.

 The benchmark is targeted toward developers during the process of optimizing
 the performance of hydroelastic contact. For example, it could be run on an
 informal basis for regression tests and to compare alternate algorithms. It
 may be removed once sufficient work has been completed on the optimization
 effort.

 <h2>Running the benchmark</h2>

 The benchmark can be executed as:

 ```
 bazel run //geometry/benchmarking:compliant_mesh_intersection_benchmark
 ```

The output will be something akin to:

 ```
Run on (64 X 3566.43 MHz CPU s)
CPU Caches:
  L1 Data 32 KiB (x32)
  L1 Instruction 32 KiB (x32)
  L2 Unified 1024 KiB (x32)
  L3 Unified 22528 KiB (x2)
Load Average: 2.31, 1.43, 2.44
---------------------------------------------------------------------------------------------------------------------------------------------  // NOLINT(*)
Benchmark                                                                                         Time             CPU    Allocs   Iterations  // NOLINT(*)
---------------------------------------------------------------------------------------------------------------------------------------------  // NOLINT(*)
CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/0/0/min_time:0.020/min_warmup_time:0.010      0.125 us        0.125 us   140.438       228197  // NOLINT(*)
CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/0/1/min_time:0.020/min_warmup_time:0.010       10.1 us         10.1 us   160.438         2687  // NOLINT(*)
CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/0/2/min_time:0.020/min_warmup_time:0.010       46.4 us         46.4 us   297.438          594  // NOLINT(*)
CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/0/3/min_time:0.020/min_warmup_time:0.010        625 us          625 us      2.7k           44  // NOLINT(*)
CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/1/0/min_time:0.020/min_warmup_time:0.010      0.116 us        0.116 us   657.188       237007  // NOLINT(*)
CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/1/1/min_time:0.020/min_warmup_time:0.010       10.4 us         10.4 us   679.188         2667  // NOLINT(*)
CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/1/2/min_time:0.020/min_warmup_time:0.010       96.9 us         96.9 us      1.0k          292  // NOLINT(*)
CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/1/3/min_time:0.020/min_warmup_time:0.010       2500 us         2500 us     11.2k           11  // NOLINT(*)
CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/2/0/min_time:0.020/min_warmup_time:0.010      0.117 us        0.117 us      1.2k       233711  // NOLINT(*)
CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/2/1/min_time:0.020/min_warmup_time:0.010       11.9 us         11.9 us      1.2k         2356  // NOLINT(*)
CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/2/2/min_time:0.020/min_warmup_time:0.010        198 us          198 us      1.9k          135  // NOLINT(*)
CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/2/3/min_time:0.020/min_warmup_time:0.010       7181 us         7181 us     36.1k            4  // NOLINT(*)
CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/3/0/min_time:0.020/min_warmup_time:0.010      0.117 us        0.117 us     10.1k       242614  // NOLINT(*)
CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/3/1/min_time:0.020/min_warmup_time:0.010       27.8 us         27.8 us     10.2k         1016  // NOLINT(*)
CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/3/2/min_time:0.020/min_warmup_time:0.010        841 us          841 us     12.7k           34  // NOLINT(*)
CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/3/3/min_time:0.020/min_warmup_time:0.010      59971 us        59969 us    433.0k            1  // NOLINT(*)

 ```

 <h4>Interpreting the benchmark</h4>

 Each line in the table represents a particular configuration of the benchmark
 parameters of the form:

 ```
 CompliantMeshIntersectionBenchmark/CompliantCompliantMesh/resolution/contact_overlap/min_time/min_warmup_time  // NOLINT(*)
 ```

   - __resolution__: Affects the resolution of the ellipsoid and sphere
     meshes. Valid values must be one of [0, 1, 2, 3], where 0 produces the
     coarsest meshes and 3 produces the finest meshes.
   - __contact_overlap__: Affects the size of the resulting contact surface by
     translating the sphere mesh relative to the ellipsoid mesh. Contact overlap
     should be one of the following enumeration values representing:
     - 0: No contact and no overlapping bounding volumes at all.
     - 1: No contact but overlapping bounding volumes.
     - 2: The minimal, or at least very small, contact surface.
     - 3: An intermediate sized contact surface.
   - __min_time__: Minimum amount of time to run the benchmark in seconds.
   - __min_warmup_time__ : Minimum amount of time to run the benchmark for
     warmup before results are collected.

 The `Time` and `CPU` columns are measures of the average time it took to
 compute the intersection. The `Iterations` indicates how often the action was
 performed to compute the average value. For more information see the [google
 benchmark documentation](https://github.com/google/benchmark).
 */
// clang-format on
using Eigen::Vector3d;
using math::RigidTransformd;

const double kElasticModulus = 1.0e5;
const double kSphereDimension = 3.;
const Vector3d kEllipsoidDimension{3.01, 3.5, 4.};
const double kResolutionHint[4] = {4., 3., 2., 1.};
const Vector3d kContactOverlapTranslation[4] = {
    Vector3d{7, 7, 7},         // 0: No overlap at all.
    Vector3d{4, 4, 4},         // 1: Overlapping bounding volumes.
    Vector3d{3.5, 3.5, 3.5},   // 2: Minimal contact surface.
    Vector3d{1.2, 1.2, 1.2}};  // 3: Intermediate sized contact surface.

class CompliantMeshIntersectionBenchmark : public benchmark::Fixture {
 public:
  CompliantMeshIntersectionBenchmark()
      : ellipsoid_{kEllipsoidDimension[0], kEllipsoidDimension[1],
                   kEllipsoidDimension[2]},
        sphere_{kSphereDimension} {}

  /* Parse arguments from the benchmark state.
  @return A tuple representing the resolution and the contact overlap.  */
  static std::tuple<int, int> ReadState(const benchmark::State& state) {
    return std::make_tuple(state.range(0), state.range(1));
  }

  /* Set up the two ellipsoid meshes and their relative transform.  */
  void SetupMeshes(const benchmark::State& state) {
    const auto [resolution, contact_overlap] = ReadState(state);
    const double resolution_hint = kResolutionHint[resolution];
    mesh_S_ =
        std::make_unique<VolumeMesh<double>>(MakeEllipsoidVolumeMesh<double>(
            ellipsoid_, resolution_hint,
            TessellationStrategy::kDenseInteriorVertices));
    field_S_ = std::make_unique<VolumeMeshFieldLinear<double, double>>(
        MakeEllipsoidPressureField<double>(ellipsoid_, mesh_S_.get(),
                                           kElasticModulus));
    mesh_R_ = std::make_unique<VolumeMesh<double>>(MakeSphereVolumeMesh<double>(
        sphere_, resolution_hint,
        TessellationStrategy::kDenseInteriorVertices));
    field_R_ = std::make_unique<VolumeMeshFieldLinear<double, double>>(
        MakeSpherePressureField<double>(sphere_, mesh_R_.get(),
                                        kElasticModulus));
    X_SR_ = RigidTransformd{kContactOverlapTranslation[contact_overlap]};
  }

  const Ellipsoid ellipsoid_;
  const Sphere sphere_;
  std::unique_ptr<VolumeMesh<double>> mesh_S_;
  std::unique_ptr<VolumeMeshFieldLinear<double, double>> field_S_;
  std::unique_ptr<VolumeMesh<double>> mesh_R_;
  std::unique_ptr<VolumeMeshFieldLinear<double, double>> field_R_;
  RigidTransformd X_SR_;
};

BENCHMARK_DEFINE_F(CompliantMeshIntersectionBenchmark, CompliantCompliantMesh)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  SetupMeshes(state);
  GeometryId id_S = GeometryId::get_new_id();
  GeometryId id_R = GeometryId::get_new_id();
  hydroelastic::SoftGeometry geo_S(
      hydroelastic::SoftMesh(std::move(mesh_S_), std::move(field_S_)));
  hydroelastic::SoftGeometry geo_R(
      hydroelastic::SoftMesh(std::move(mesh_R_), std::move(field_R_)));

  std::unique_ptr<ContactSurface<double>> surface_SR;
  for (auto _ : state) {
    surface_SR = CalcCompliantCompliant(
        geo_S, RigidTransformd::Identity(), id_S, geo_R, X_SR_, id_R,
        HydroelasticContactRepresentation::kPolygon);
  }
}
// clang-format off
BENCHMARK_REGISTER_F(CompliantMeshIntersectionBenchmark, CompliantCompliantMesh)
    ->Unit(benchmark::kMicrosecond)
    ->MinTime(0.02)
    ->MinWarmUpTime(0.01)
    ->Args({0, 0})
    ->Args({0, 1})
    ->Args({0, 2})
    ->Args({0, 3})
    ->Args({1, 0})
    ->Args({1, 1})
    ->Args({1, 2})
    ->Args({1, 3})
    ->Args({2, 0})
    ->Args({2, 1})
    ->Args({2, 2})
    ->Args({2, 3})
    ->Args({3, 0})
    ->Args({3, 1})
    ->Args({3, 2})
    ->Args({3, 3});
// clang-format on
}  // namespace internal
}  // namespace geometry
}  // namespace drake
