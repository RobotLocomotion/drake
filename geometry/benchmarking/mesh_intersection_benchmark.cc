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

 It computes the contact surface formed from their intersection both using and
 not using broad-phase culling (via a bounding volume hierarchy). Arguments
 include:
 - __resolution hint__: Guides the level of mesh refinement. See
   @ref AddRigidHydroelasticProperties() for more details.
 - __penetration offset__: How much to offset the penetration between the two
   meshes in meters. By default the meshes start with their centers at the same
   position.
 - __rotation factor__: Affects how much the meshes are in axis alignment. The
   given scalar is used as a multiplicative factor of PI/6 around each axis.

 The benchmark is targeted toward developers during the process of optimizing
 the performance of hydroelastic contact. For example, it could be run on an
 informal basis for regression tests and to compare alternate algorithms. It
 may be removed once sufficient work has been completed on the optimization
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
MeshIntersectionBenchmark/WithoutBVH/1/0/0/min_time:2.000        530 ms          530 ms            5
MeshIntersectionBenchmark/WithoutBVH/2/0/0/min_time:2.000       18.8 ms         18.8 ms          149
MeshIntersectionBenchmark/WithoutBVH/3/0/0/min_time:2.000       19.2 ms         19.2 ms          147
MeshIntersectionBenchmark/WithoutBVH/4/0/0/min_time:2.000      0.792 ms        0.792 ms         3501
MeshIntersectionBenchmark/WithoutBVH/2/1/0/min_time:2.000       18.2 ms         18.2 ms          155
MeshIntersectionBenchmark/WithoutBVH/2/2/0/min_time:2.000       17.1 ms         17.1 ms          165
MeshIntersectionBenchmark/WithoutBVH/2/0/1/min_time:2.000       19.1 ms         19.1 ms          149
MeshIntersectionBenchmark/WithoutBVH/2/0/2/min_time:2.000       18.8 ms         18.8 ms          147
MeshIntersectionBenchmark/WithoutBVH/2/1/1/min_time:2.000       18.4 ms         18.4 ms          152
MeshIntersectionBenchmark/WithBVH/1/0/0/min_time:2.000          11.3 ms         11.3 ms          250
MeshIntersectionBenchmark/WithBVH/2/0/0/min_time:2.000          2.68 ms         2.68 ms          998
MeshIntersectionBenchmark/WithBVH/3/0/0/min_time:2.000          2.62 ms         2.62 ms         1041
MeshIntersectionBenchmark/WithBVH/4/0/0/min_time:2.000         0.570 ms        0.570 ms         4994
MeshIntersectionBenchmark/WithBVH/2/1/0/min_time:2.000          1.61 ms         1.61 ms         1745
MeshIntersectionBenchmark/WithBVH/2/2/0/min_time:2.000         0.820 ms        0.820 ms         3500
MeshIntersectionBenchmark/WithBVH/2/0/1/min_time:2.000          2.50 ms         2.50 ms         1124
MeshIntersectionBenchmark/WithBVH/2/0/2/min_time:2.000          2.28 ms         2.28 ms         1274
MeshIntersectionBenchmark/WithBVH/2/1/1/min_time:2.000          2.19 ms         2.19 ms         1302

 ```

 <h4>Interpreting the benchmark</h4>

 Each line in the table represents a particular configuration of the benchmark
 parameters of the form:

 ```
 BenchmarkName/TestName/resolution_hint/penetration_offset/rotation_factor/min_time
 ```

   - __Benchmarkame__: `MeshIntersectionBenchmark` for all results in this
      executable.
   - __TestName__: One of
     - __WithoutBVH__: Computes the intersection without broad-phase culling.
     - __WithBVH__: Computes the intersection with broad-phase culling.
   - __resolution_hint__: Because the mesh's maximum measure is 4m, providing a
     resolution hint of 4m will produce the coarsest mesh possible. Each time
     you cut that value in half, the resolution will increase. The resolution
     hint must be greater than 0. In this benchmark, the resolution hint is
     limited to integer values due to Google Benchmark's argument parsing.
   - __penetration_offset__: The two meshes are initialized such that their
     origins share the same position. Assuming no rotation, this results in
     maximal overlap for identical meshes. The given offset in meters is
     then used to translate the rigid mesh along each axis to reduce
     penetration. This forms the translation component of the transformation
     matrix from the soft mesh to the rigid mesh, X_SR.
   - __rotation_factor__: How much rotation to offset between the soft and the
     rigid mesh to increase axis misalignment, i.e. for creating X_SR, since by
     default the meshes are intialized to be axis aligned. Given an offset,
     int `i`,  the rotation is a multiplicative factor of `i*π/6` radians around
     each of the geometry's axes. Note: we use this integer factor instead of
     directly specifying the rotation because Google Benchmark does not accept
     doubles as arguments.
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
     - For a resolution hint parameter of 2, broad-phase culling improves
       performance by a factor of ~7X (see row 2 vs row 11), while at a
       resolution hint parameter of 1 (a finer mesh), it improves by a factor
       of ~48X (see row 1 vs row 10).
   - As expected, the BVH is increasingly effective as penetration is reduced.
     - At a penetration offset parameter of 1, broad-phase culling improves
       performance by a factor of ~11X (see row 5 vs row 14), while at a
       penetration offset parameter of 2 (less penetration), it improves
       performance by a factor of ~20X (see row 6 vs row 15).
   - The offset in axis alignment has an apparently negligible impact on
     performance.
 */

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;

const double kElasticModulus = 1.0e5;

class MeshIntersectionBenchmark : public benchmark::Fixture {
 public:
  MeshIntersectionBenchmark()
      : ellipsoid_{2., 3., 4.},
        mesh_S_(MakeEllipsoidVolumeMesh<double>(ellipsoid_, 1)),
        field_S_(MakeEllipsoidPressureField<double>(ellipsoid_, &mesh_S_,
                                                    kElasticModulus)),
        mesh_R_(MakeEllipsoidSurfaceMesh<double>(ellipsoid_, 1)) {}

  /** Parse arguments from the benchmark state.
  @return A tuple representing the resolution hint, the penetration depth, and
          the rotation factor.  */
  static std::tuple<int, double, int> ReadState(
      const benchmark::State& state) {
    return std::make_tuple(state.range(0), static_cast<double>(state.range(1)),
                           state.range(2));
  }

  void SetupMeshes(const benchmark::State& state) {
    const auto [edge_length, translation, rotation_factor] = ReadState(state);
    mesh_S_ = MakeEllipsoidVolumeMesh<double>(ellipsoid_, edge_length);
    field_S_ = MakeEllipsoidPressureField<double>(ellipsoid_, &mesh_S_,
                                                      kElasticModulus);
    mesh_R_ = MakeEllipsoidSurfaceMesh<double>(ellipsoid_, edge_length);
    X_SR_ = RigidTransformd{
        RollPitchYawd(M_PI / 6 * rotation_factor, M_PI / 6 * rotation_factor,
                      M_PI / 6 * rotation_factor),
        Vector3d{translation, translation, translation}};
  }

  Ellipsoid ellipsoid_;
  VolumeMesh<double> mesh_S_;
  VolumeMeshFieldLinear<double, double> field_S_;
  SurfaceMesh<double> mesh_R_;
  RigidTransformd X_SR_;
};

BENCHMARK_DEFINE_F(MeshIntersectionBenchmark, WithoutBVH)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  SetupMeshes(state);
  std::unique_ptr<SurfaceMesh<double>> surface_SR;
  std::unique_ptr<SurfaceMeshFieldLinear<double, double>> e_SR;
  for (auto _ : state) {
    SampleVolumeFieldOnSurface(field_S_, mesh_R_, X_SR_, &surface_SR, &e_SR);
  }
}
BENCHMARK_REGISTER_F(MeshIntersectionBenchmark, WithoutBVH)
    ->Unit(benchmark::kMillisecond)
    ->MinTime(2)
    ->Args({1, 0, 0})   // 1m resolution hint, 0 penetration offset, 0 rotation factor. NOLINT
    ->Args({2, 0, 0})   // 2m resolution hint, 0 penetration offset, 0 rotation factor. NOLINT
    ->Args({3, 0, 0})   // 3m resolution hint, 0 penetration offset, 0 rotation factor. NOLINT
    ->Args({4, 0, 0})   // 4m resolution hint, 0 penetration offset, 0 rotation factor. NOLINT
    ->Args({2, 1, 0})   // 2m resolution hint, 1 penetration offset, 0 rotation factor. NOLINT
    ->Args({2, 2, 0})   // 2m resolution hint, 2 penetration offset, 0 rotation factor. NOLINT
    ->Args({2, 0, 1})   // 2m resolution hint, 0 penetration offset, pi/6 rotation factor. NOLINT
    ->Args({2, 0, 2})   // 2m resolution hint, 0 penetration offset, pi/3 rotation factor. NOLINT
    ->Args({2, 1, 1});  // 2m resolution hint, 1 penetration offset, pi/6 rotation factor. NOLINT

BENCHMARK_DEFINE_F(MeshIntersectionBenchmark, WithBVH)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  SetupMeshes(state);
  const auto bvh_S = BoundingVolumeHierarchy<VolumeMesh<double>>(mesh_S_);
  const auto bvh_R = BoundingVolumeHierarchy<SurfaceMesh<double>>(mesh_R_);
  std::unique_ptr<SurfaceMesh<double>> surface_SR;
  std::unique_ptr<SurfaceMeshFieldLinear<double, double>> e_SR;
  for (auto _ : state) {
    SampleVolumeFieldOnSurface(field_S_, bvh_S, mesh_R_, bvh_R, X_SR_,
                               &surface_SR, &e_SR);
  }
}
BENCHMARK_REGISTER_F(MeshIntersectionBenchmark, WithBVH)
    ->Unit(benchmark::kMillisecond)
    ->MinTime(2)
    ->Args({1, 0, 0})   // 1m resolution hint, 0 penetration offset, 0 rotation. NOLINT
    ->Args({2, 0, 0})   // 2m resolution hint, 0 penetration offset, 0 rotation factor. NOLINT
    ->Args({3, 0, 0})   // 3m resolution hint, 0 penetration offset, 0 rotation factor. NOLINT
    ->Args({4, 0, 0})   // 4m resolution hint, 0 penetration offset, 0 rotation factor. NOLINT
    ->Args({2, 1, 0})   // 2m resolution hint, 1 penetration offset, 0 rotation factor. NOLINT
    ->Args({2, 2, 0})   // 2m resolution hint, 2 penetration offset, 0 rotation factor. NOLINT
    ->Args({2, 0, 1})   // 2m resolution hint, 0 penetration offset, pi/6 rotation factor. NOLINT
    ->Args({2, 0, 2})   // 2m resolution hint, 0 penetration offset, pi/3 rotation factor. NOLINT
    ->Args({2, 1, 1});  // 2m resolution hint, 1 penetration offset, pi/6 rotation factor. NOLINT

}  // namespace internal
}  // namespace geometry
}  // namespace drake

int main(int argc, char** argv) {
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
