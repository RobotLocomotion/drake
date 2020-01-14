#include <benchmark/benchmark.h>

#include "drake/geometry/proximity/make_ellipsoid_field.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
#include "drake/geometry/proximity/mesh_intersection.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/** @defgroup mesh_intersection_benchmarks Mesh Intersection Benchmarks
 @ingroup proximity_queries

 The benchmark compares mesh intersection techniques between soft and rigid
 meshes.

 It computes the contact surface formed from their intersection both with and
 without using broad-phase culling (via a bounding volume hierarchy). Arguments
 include:
 - __edge_length__: Defines the length of each edge in the mesh. A smaller edge
   length results in higher resolution.
 - __overlap__: How much the two meshes overlap using a translation offset.
 - __axis alignment__: How much the meshes are in axis alignment using a
   rotation offset.

 The benchmark is targeted toward developers during the process of optimizing
 the performance of hydroelastic contact. For example, it could be run as an
 informal basis for regression tests and to compare alternate algorithms. It
 may be removed once sufficient work has been completed on the hydroelastic
 effort.

 <h2>Running the benchmark</h2>

 The benchmark can be executed as:

 ```
 bazel run //geometry/benchmarking:mesh_intersection_benchmark
 ```

 The output will be something akin to:

 ```
Run on (56 X 3500 MHz CPU s)
CPU Caches:
  L1 Data 32K (x28)
  L1 Instruction 32K (x28)
  L2 Unified 256K (x28)
  L3 Unified 35840K (x2)
Load Average: 21.52, 41.04, 26.99
***WARNING*** CPU scaling is enabled, the benchmark real time measurements may be noisy and will incur extra overhead.
----------------------------------------------------------------------------------------------------
Benchmark                                                          Time             CPU   Iterations
----------------------------------------------------------------------------------------------------
MeshIntersectionBenchmark/WithoutBVH/1/0/0/min_time:2.000        524 ms          524 ms            5
MeshIntersectionBenchmark/WithoutBVH/2/0/0/min_time:2.000       19.9 ms         19.9 ms          143
MeshIntersectionBenchmark/WithoutBVH/3/0/0/min_time:2.000       19.0 ms         19.0 ms          111
MeshIntersectionBenchmark/WithoutBVH/4/0/0/min_time:2.000      0.792 ms        0.792 ms         3577
MeshIntersectionBenchmark/WithoutBVH/2/1/0/min_time:2.000       18.4 ms         18.4 ms          154
MeshIntersectionBenchmark/WithoutBVH/2/2/0/min_time:2.000       17.0 ms         17.0 ms          166
MeshIntersectionBenchmark/WithoutBVH/2/0/1/min_time:2.000       19.0 ms         19.0 ms          148
MeshIntersectionBenchmark/WithoutBVH/2/0/2/min_time:2.000       18.9 ms         18.9 ms          148
MeshIntersectionBenchmark/WithoutBVH/2/1/1/min_time:2.000       18.7 ms         18.7 ms          150
MeshIntersectionBenchmark/WithBVH/1/0/0/min_time:2.000          11.0 ms         11.0 ms          256
MeshIntersectionBenchmark/WithBVH/2/0/0/min_time:2.000          2.63 ms         2.63 ms         1078
MeshIntersectionBenchmark/WithBVH/3/0/0/min_time:2.000          2.64 ms         2.64 ms         1052
MeshIntersectionBenchmark/WithBVH/4/0/0/min_time:2.000         0.566 ms        0.566 ms         4942
MeshIntersectionBenchmark/WithBVH/2/1/0/min_time:2.000          1.62 ms         1.62 ms         1729
MeshIntersectionBenchmark/WithBVH/2/2/0/min_time:2.000         0.807 ms        0.807 ms         3468
MeshIntersectionBenchmark/WithBVH/2/0/1/min_time:2.000          2.16 ms         2.16 ms         1278
MeshIntersectionBenchmark/WithBVH/2/0/2/min_time:2.000          2.18 ms         2.18 ms         1302
MeshIntersectionBenchmark/WithBVH/2/1/1/min_time:2.000          1.91 ms         1.91 ms         1463

 ```

 <h4>Interpreting the benchmark</h4>

 Each line in the table represents a particular configuration of the benchmark
 parameters of the form:

 ```
 BenchmarkName/TestName/edge_length/translation_offset/rotation_offset/min_time
 ```

   - __Benchmarkname__: `MeshIntersectionBenchmark` for all results in this
      executable.
   - __TestName__: One of
     - __WithoutBVH__: Computes the intersection without broad-phase culling.
     - __WithBVH__: Computes the intersection with broad-phase culling.
   - __edge_length__: The length of the edge. Note that since we've defined the
     mesh's max dimension in this benchmark as 4, then an edge length of 4 gives
     the coarsest mesh possible with a resolution of 4/4 = 1. An edge length of
     2 equates to a resolution of 2/4 = 0.5.
   - __translation_offset__: How much translation to offset between the soft and
     the rigid mesh to reduce overlap, i.e. for creating X_SR.
   - __rotation_offset__: How much rotation to offset between the soft and the
     rigid mesh to increase axis misalignment, i.e. for creating X_SR. This
     number is used as a multiplicative factor of PI/6. Note: we use this factor
     instead of directly specifying the rotation because Google Benchmark
     does not accept doubles as arguments.
   - __min_time__: Minimum amount of time to run the benchmark in seconds. This
     needs to be specified for tests that run long in order to get enough
     iterations.

 The `Time` and `CPU` columns are measures of the average time it took to
 compute the intersection. The `Iterations` indicates how often the action was
 performed to compute the average value. For more information see the [google
 benchmark documentation](https://github.com/google/benchmark).

 Now we can analyze the example output and draw some example inferences (not a
 complete set of valid inferences):

   - The BVH is increasingly effective as mesh resolution increases.
     - At 0.5 resolution it improves performance by a factor of ~7 while at 0.25
       resolution it improves performance by a factor of ~48.
   - As expected, the BVH is increasingly effective as overlap is reduced.
     - At a translation offset of 1 it improves performance by a factor of ~11
       while at 0.25 at a translation offset of 2 it improves performance by a
       factor of ~20.
   - The offset in axis alignment has an apparently negligible impact on
     performance.
 */

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;

const double kElasticModulus = 1.0e5;

class MeshIntersectionBenchmark : public benchmark::Fixture {
 public:
  /** Parse arguments from the benchmark state.
  @return A tuple representing the edge length and the amount to translate for
          varying overlap.  */
  static std::tuple<int, double, int> ReadState(
      const benchmark::State& state) {
    return std::make_tuple(state.range(0), static_cast<double>(state.range(1)),
                           state.range(2));
  }
};

BENCHMARK_DEFINE_F(MeshIntersectionBenchmark, WithoutBVH)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
    const Ellipsoid ellipsoid(2., 3., 4.);
    auto[edge_length, translation, rotation_factor] = ReadState(state);
    auto mesh_S = MakeEllipsoidVolumeMesh<double>(ellipsoid, edge_length);
    auto field_S =
        MakeEllipsoidPressureField<double>(ellipsoid, &mesh_S, kElasticModulus);
    auto mesh_R =
        MakeEllipsoidSurfaceMesh<double>(Ellipsoid(2., 3., 4.), edge_length);
    std::unique_ptr<SurfaceMesh<double>> surface_SR;
    std::unique_ptr<SurfaceMeshFieldLinear<double, double>> e_SR;
    const auto X_SR = RigidTransformd{
        RollPitchYawd(M_PI / 6 * rotation_factor, M_PI / 6 * rotation_factor,
                      M_PI / 6 * rotation_factor),
        Vector3d{translation, translation, translation}};
    for (auto _ : state) {
      SampleVolumeFieldOnSurface(field_S, mesh_R, X_SR, &surface_SR, &e_SR);
    }
}
BENCHMARK_REGISTER_F(MeshIntersectionBenchmark, WithoutBVH)
    ->Unit(benchmark::kMillisecond)
    ->MinTime(2)
    ->Args({1, 0, 0})   // 1/4 edge length, 0 translation, 0 rotation.
    ->Args({2, 0, 0})   // 2/4 edge length, 0 translation, 0 rotation.
    ->Args({3, 0, 0})   // 3/4 edge length, 0 translation, 0 rotation.
    ->Args({4, 0, 0})   // 4/4 edge length, 0 translation, 0 rotation.
    ->Args({2, 1, 0})   // 2/4 edge length, 1 translation, 0 rotation.
    ->Args({2, 2, 0})   // 2/4 edge length, 2 translation, 0 rotation.
    ->Args({2, 0, 1})   // 2/4 edge length, 0 translation, pi/6 rotation.
    ->Args({2, 0, 2})   // 2/4 edge length, 0 translation, pi/3 rotation.
    ->Args({2, 1, 1});  // 2/4 edge length, 1 translation, pi/6 rotation.

BENCHMARK_DEFINE_F(MeshIntersectionBenchmark, WithBVH)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  const Ellipsoid ellipsoid(2., 3., 4.);
  auto[edge_length, translation, rotation_factor] = ReadState(state);
  auto mesh_S = MakeEllipsoidVolumeMesh<double>(ellipsoid, edge_length);
  auto bvh_S = BoundingVolumeHierarchy<VolumeMesh<double>>(mesh_S);
  auto field_S =
      MakeEllipsoidPressureField<double>(ellipsoid, &mesh_S, kElasticModulus);
  auto mesh_R =
      MakeEllipsoidSurfaceMesh<double>(Ellipsoid(2., 3., 4.), edge_length);
  auto bvh_R = BoundingVolumeHierarchy<SurfaceMesh<double>>(mesh_R);
  std::unique_ptr<SurfaceMesh<double>> surface_SR;
  std::unique_ptr<SurfaceMeshFieldLinear<double, double>> e_SR;
  const auto X_SR = RigidTransformd{
      RollPitchYawd(M_PI / 3 * rotation_factor, M_PI / 3 * rotation_factor,
                    M_PI / 3 * rotation_factor),
      Vector3d{translation, translation, translation}};
  for (auto _ : state) {
    SampleVolumeFieldOnSurface(field_S, bvh_S, mesh_R, bvh_R, X_SR, &surface_SR,
                               &e_SR);
  }
}
BENCHMARK_REGISTER_F(MeshIntersectionBenchmark, WithBVH)
    ->Unit(benchmark::kMillisecond)
    ->MinTime(2)
    ->Args({1, 0, 0})   // 1/4 edge length, 0 translation, 0 rotation.
    ->Args({2, 0, 0})   // 2/4 edge length, 0 translation, 0 rotation.
    ->Args({3, 0, 0})   // 3/4 edge length, 0 translation, 0 rotation.
    ->Args({4, 0, 0})   // 4/4 edge length, 0 translation, 0 rotation.
    ->Args({2, 1, 0})   // 2/4 edge length, 1 translation, 0 rotation.
    ->Args({2, 2, 0})   // 2/4 edge length, 2 translation, 0 rotation.
    ->Args({2, 0, 1})   // 2/4 edge length, 0 translation, pi/6 rotation.
    ->Args({2, 0, 2})   // 2/4 edge length, 0 translation, pi/3 rotation.
    ->Args({2, 1, 1});  // 2/4 edge length, 1 translation, pi/6 rotation.

}  // namespace internal
}  // namespace geometry
}  // namespace drake

int main(int argc, char** argv) {
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
