#include <benchmark/benchmark.h>
#include "fmt/format.h"

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

 It computes the contact surface formed from the intersection of two identical
 ellipsoids both using and not using broad-phase culling (via a bounding volume
 hierarchy). Arguments include:
 - __resolution hint__: Guides the level of mesh refinement. See
   AddRigidHydroelasticProperties() for more details.
 - __contact overlap__: An enumeration in the integer range from 0 to 4 that
   correlates with the size of the resultant contact surface between the two
   meshes. Note that axis misalignment (i.e. setting a non-zero rotation factor)
   can also affect the size of the resultant contact surface, so for the final
   accurate size, refer to the report at the bottom of the benchmark output.
 - __rotation factor__: Affects how much the meshes are in axis alignment. The
   given scalar is used as a multiplicative factor of PI/4 around the shortest
   axis of the ellipsoid.

 For more details on arguments, see <b>Interpreting the benchmark</b> below.

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
MeshIntersectionBenchmark/WithoutBVH/1/0/0/min_time:2.000        544 ms          544 ms            5   [1]
MeshIntersectionBenchmark/WithoutBVH/2/0/0/min_time:2.000       19.2 ms         19.2 ms          146   [2]
MeshIntersectionBenchmark/WithoutBVH/3/0/0/min_time:2.000       19.3 ms         19.3 ms          137   [3]
MeshIntersectionBenchmark/WithoutBVH/4/0/0/min_time:2.000      0.797 ms        0.797 ms         3455   [4]
MeshIntersectionBenchmark/WithoutBVH/2/1/0/min_time:2.000       17.9 ms         17.9 ms          157   [5]
MeshIntersectionBenchmark/WithoutBVH/2/2/0/min_time:2.000       16.2 ms         16.2 ms          173   [6]
MeshIntersectionBenchmark/WithoutBVH/2/3/0/min_time:2.000       15.9 ms         15.9 ms          175   [7]
MeshIntersectionBenchmark/WithoutBVH/2/4/0/min_time:2.000       15.3 ms         15.3 ms          184   [8]
MeshIntersectionBenchmark/WithoutBVH/2/0/1/min_time:2.000       19.1 ms         19.1 ms          146   [9]
MeshIntersectionBenchmark/WithoutBVH/2/0/2/min_time:2.000       19.3 ms         19.3 ms          144   [10]
MeshIntersectionBenchmark/WithoutBVH/2/0/3/min_time:2.000       19.0 ms         19.0 ms          146   [11]
MeshIntersectionBenchmark/WithoutBVH/2/1/1/min_time:2.000       17.8 ms         17.8 ms          158   [12]
MeshIntersectionBenchmark/WithoutBVH/2/2/2/min_time:2.000       16.1 ms         16.1 ms          174   [13]
MeshIntersectionBenchmark/WithBVH/1/0/0/min_time:2.000          11.2 ms         11.2 ms          254   [14]
MeshIntersectionBenchmark/WithBVH/2/0/0/min_time:2.000          2.65 ms         2.65 ms         1069   [15]
MeshIntersectionBenchmark/WithBVH/3/0/0/min_time:2.000          2.64 ms         2.64 ms         1053   [16]
MeshIntersectionBenchmark/WithBVH/4/0/0/min_time:2.000         0.574 ms        0.574 ms         4921   [17]
MeshIntersectionBenchmark/WithBVH/2/1/0/min_time:2.000          1.39 ms         1.39 ms         1990   [18]
MeshIntersectionBenchmark/WithBVH/2/2/0/min_time:2.000         0.101 ms        0.101 ms        27725   [19]
MeshIntersectionBenchmark/WithBVH/2/3/0/min_time:2.000         0.048 ms        0.048 ms        57236   [20]
MeshIntersectionBenchmark/WithBVH/2/4/0/min_time:2.000         0.000 ms        0.000 ms     54022591   [21]
MeshIntersectionBenchmark/WithBVH/2/0/1/min_time:2.000          2.41 ms         2.41 ms         1165   [22]
MeshIntersectionBenchmark/WithBVH/2/0/2/min_time:2.000          2.48 ms         2.48 ms         1130   [23]
MeshIntersectionBenchmark/WithBVH/2/0/3/min_time:2.000          2.45 ms         2.45 ms         1159   [24]
MeshIntersectionBenchmark/WithBVH/2/1/1/min_time:2.000          1.40 ms         1.40 ms         1968   [25]
MeshIntersectionBenchmark/WithBVH/2/2/2/min_time:2.000         0.089 ms        0.089 ms        31026   [26]
Resulting contact surface sizes:
 - WithoutBVH/1/0/0: 84.68 m^2, 1160 triangles
 - WithoutBVH/2/0/0: 98.91 m^2, 360 triangles
 - WithoutBVH/3/0/0: 98.91 m^2, 360 triangles
 - WithoutBVH/4/0/0: 76.11 m^2, 72 triangles
 - WithoutBVH/2/1/0: 27.92 m^2, 1624 triangles
 - WithoutBVH/2/2/0: 0.48 m^2, 82 triangles
 - WithoutBVH/2/3/0: 0.00 m^2, 0 triangles
 - WithoutBVH/2/4/0: 0.00 m^2, 0 triangles
 - WithoutBVH/2/0/1: 51.03 m^2, 2164 triangles
 - WithoutBVH/2/0/2: 49.27 m^2, 2092 triangles
 - WithoutBVH/2/0/3: 47.65 m^2, 2040 triangles
 - WithoutBVH/2/1/1: 27.24 m^2, 1556 triangles
 - WithoutBVH/2/2/2: 0.05 m^2, 74 triangles
 - WithBVH/1/0/0: 84.68 m^2, 1160 triangles
 - WithBVH/2/0/0: 98.91 m^2, 360 triangles
 - WithBVH/3/0/0: 98.91 m^2, 360 triangles
 - WithBVH/4/0/0: 76.11 m^2, 72 triangles
 - WithBVH/2/1/0: 27.92 m^2, 1624 triangles
 - WithBVH/2/2/0: 0.48 m^2, 82 triangles
 - WithBVH/2/3/0: 0.00 m^2, 0 triangles
 - WithBVH/2/4/0: 0.00 m^2, 0 triangles
 - WithBVH/2/0/1: 51.03 m^2, 2164 triangles
 - WithBVH/2/0/2: 49.27 m^2, 2092 triangles
 - WithBVH/2/0/3: 47.65 m^2, 2040 triangles
 - WithBVH/2/1/1: 27.24 m^2, 1556 triangles
 - WithBVH/2/2/2: 0.05 m^2, 74 triangles

 ```
 Note: the terminating line numbers [xx] have been added for referencing in
 discussions below.

 <h4>Interpreting the benchmark</h4>

 Each line in the table represents a particular configuration of the benchmark
 parameters of the form:

 ```
 MeshIntersectionBenchmark/TestName/resolution_hint/contact_overlap/rotation_factor/min_time
 ```

   - __TestName__: One of
     - __WithoutBVH__: Computes the intersection without broad-phase culling.
     - __WithBVH__: Computes the intersection with broad-phase culling.
   - __resolution_hint__: Because the mesh's maximum measure is 4m, providing a
     resolution hint of 4m will produce the coarsest mesh possible. Each time
     you cut that value in half, the resolution will increase. The resolution
     hint must be greater than 0. In this benchmark, the resolution hint is
     limited to integer values due to Google Benchmark's argument parsing.
   - __contact_overlap__: Affects the size of the resulting contact surface by
     translating the rigid mesh relative to the soft mesh. Contact overlap
     should be one of the following enumeration values representing:
     - 0: The maximal contact surface (in area).
     - 1: A lesser contact surface.
     - 2: The minimal, or at least very small, contact surface.
     - 3: No contact but overlapping bounding volumes.
     - 4: No contact and no overlapping bounding volumes at all.
   - __rotation_factor__: How much rotation to offset between the soft and the
     rigid mesh to increase axis misalignment. Given an offset, int `i`, the
     resulting rotation is calculated as `i / max_factor * π/4` radians around
     the geometry's shortest axis, in this case, the x-axis. When rotation
     factor is zero, they are perfectly aligned, as rotation factor increases,
     the dot product between the two sets of misaligned axes are minimized. We
     set the max_factor to 3, so valid rotation factors are in the range
     [0, 1, 2, 3]. Note: we use this integer factor instead of directly
     specifying the rotation because Google Benchmark does not accept doubles
     as arguments.
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
       performance by a factor of ~7X (see [2] vs [15]), while at a
       resolution hint parameter of 1 (a finer mesh), it improves by a factor
       of ~48X (see [1] vs [14]).
   - As expected, the BVH is increasingly effective as contact overlap is
     reduced.
     - At a contact overlap parameter of 0, i.e. a maximal contact surface,
       broad-phase culling improves performance by a factor of ~7X (see [2] vs
       [15]), while at a contact overlap parameter of 2, i.e. a minimal contact
       surface, it improves performance by a factor of ~160X (see [6] vs [19]).
   - The offset in axis alignment has an apparently negligible impact on
     performance.
 */

using Eigen::AngleAxis;
using Eigen::Vector3d;
using math::RigidTransformd;

const double kElasticModulus = 1.0e5;
const double kMaxRotationFactor = 3.;
const Vector3d kEllipsoidDimension{2., 3., 4.};
const Vector3d kContactOverlapTranslation[5] = {
    Vector3d{0, 0, 0},           // 0: Maximal contact surface.
    Vector3d{1.5, 1.5, 1.5},     // 1: Intermediate sized contact surface.
    Vector3d{2.95, 2.95, 2.95},  // 2: Minimal contact surface.
    Vector3d{3.2, 3.2, 3.2},     // 3: Overlapping bounding volumes.
    Vector3d{7, 7, 7}};          // 4: No overlap at all.

class MeshIntersectionBenchmark : public benchmark::Fixture {
 public:
  MeshIntersectionBenchmark()
      : ellipsoid_{kEllipsoidDimension[0], kEllipsoidDimension[1],
                   kEllipsoidDimension[2]},
        mesh_S_(MakeEllipsoidVolumeMesh<double>(ellipsoid_, 1)),
        field_S_(MakeEllipsoidPressureField<double>(ellipsoid_, &mesh_S_,
                                                    kElasticModulus)),
        mesh_R_(MakeEllipsoidSurfaceMesh<double>(ellipsoid_, 1)) {}

  /** Parse arguments from the benchmark state.
  @return A tuple representing the resolution hint, the contact overlap, and
          the rotation factor.  */
  static std::tuple<int, int, int> ReadState(
      const benchmark::State& state) {
    return std::make_tuple(state.range(0), state.range(1), state.range(2));
  }

  /** Set up the two ellipsoid meshes and their relative transform.  */
  void SetupMeshes(const benchmark::State& state) {
    const auto [resolution_hint, contact_overlap, rotation_factor] =
        ReadState(state);
    mesh_S_ = MakeEllipsoidVolumeMesh<double>(ellipsoid_, resolution_hint);
    field_S_ = MakeEllipsoidPressureField<double>(ellipsoid_, &mesh_S_,
                                                  kElasticModulus);
    mesh_R_ = MakeEllipsoidSurfaceMesh<double>(ellipsoid_, resolution_hint);
    X_SR_ = RigidTransformd{
        AngleAxis(rotation_factor / kMaxRotationFactor * M_PI / 4,
                  Vector3d{1, 0, 0}),
        kContactOverlapTranslation[contact_overlap]};
  }

  /** Record the size of the resulting contact surface between the two meshes
   for reporting later.  */
  void RecordContactSurfaceResult(const SurfaceMesh<double>* surface_SR,
                                  const std::string& test_name,
                                  const benchmark::State& state) {
    const int num_elements =
        surface_SR == nullptr ? 0 : surface_SR->num_elements();
    const double area = surface_SR == nullptr ? 0 : surface_SR->total_area();
    const auto [resolution_hint, contact_overlap, rotation_factor] =
        ReadState(state);
    const std::string result_key =
        fmt::format("{}/{}/{}/{}", test_name, resolution_hint, contact_overlap,
                    rotation_factor);
    if (contact_surface_result_keys.find(result_key) ==
        contact_surface_result_keys.end()) {
      contact_surface_result_output.push_back(fmt::format(
          "{}: {:.2f} m^2, {} triangles", result_key, area, num_elements));
      contact_surface_result_keys.insert(result_key);
    }
  }

  // Keep track of the number of elements in the resulting contact surface. We
  // use a static set because Google Benchmark runs these benchmarks multiple
  // times with unique instances of the fixture, and we want to avoid duplicate
  // output when we display this at the end. We use a vector to store the
  // actual output so that the order matches that of the benchmark results.
  static std::set<std::string> contact_surface_result_keys;
  static std::vector<std::string> contact_surface_result_output;

  Ellipsoid ellipsoid_;
  VolumeMesh<double> mesh_S_;
  VolumeMeshFieldLinear<double, double> field_S_;
  SurfaceMesh<double> mesh_R_;
  RigidTransformd X_SR_;
};
std::set<std::string> MeshIntersectionBenchmark::contact_surface_result_keys;
std::vector<std::string>
    MeshIntersectionBenchmark::contact_surface_result_output;

BENCHMARK_DEFINE_F(MeshIntersectionBenchmark, WithoutBVH)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  SetupMeshes(state);
  std::unique_ptr<SurfaceMesh<double>> surface_SR;
  std::unique_ptr<SurfaceMeshFieldLinear<double, double>> e_SR;
  for (auto _ : state) {
    SampleVolumeFieldOnSurface(field_S_, mesh_R_, X_SR_, &surface_SR, &e_SR);
  }
  RecordContactSurfaceResult(surface_SR.get(), "WithoutBVH", state);
}
BENCHMARK_REGISTER_F(MeshIntersectionBenchmark, WithoutBVH)
    ->Unit(benchmark::kMillisecond)
    ->MinTime(2)
    ->Args({1, 0, 0})   // 1m resolution hint, 0 contact overlap, 0 rotation factor. NOLINT
    ->Args({2, 0, 0})   // 2m resolution hint, 0 contact overlap, 0 rotation factor. NOLINT
    ->Args({3, 0, 0})   // 3m resolution hint, 0 contact overlap, 0 rotation factor. NOLINT
    ->Args({4, 0, 0})   // 4m resolution hint, 0 contact overlap, 0 rotation factor. NOLINT
    ->Args({2, 1, 0})   // 2m resolution hint, 1 contact overlap, 0 rotation factor. NOLINT
    ->Args({2, 2, 0})   // 2m resolution hint, 2 contact overlap, 0 rotation factor. NOLINT
    ->Args({2, 3, 0})   // 2m resolution hint, 3 contact overlap, 0 rotation factor. NOLINT
    ->Args({2, 4, 0})   // 2m resolution hint, 4 contact overlap, 0 rotation factor. NOLINT
    ->Args({2, 0, 1})   // 2m resolution hint, 0 contact overlap, 1 rotation factor. NOLINT
    ->Args({2, 0, 2})   // 2m resolution hint, 0 contact overlap, 2 rotation factor. NOLINT
    ->Args({2, 0, 3})   // 2m resolution hint, 0 contact overlap, 3 rotation factor. NOLINT
    ->Args({2, 1, 1})   // 2m resolution hint, 1 contact overlap, 1 rotation factor. NOLINT
    ->Args({2, 2, 2});  // 2m resolution hint, 2 contact overlap, 2 rotation factor. NOLINT

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
  RecordContactSurfaceResult(surface_SR.get(), "WithBVH", state);
}
BENCHMARK_REGISTER_F(MeshIntersectionBenchmark, WithBVH)
    ->Unit(benchmark::kMillisecond)
    ->MinTime(2)
    ->Args({1, 0, 0})   // 1m resolution hint, 0 contact overlap, 0 rotation factor. NOLINT
    ->Args({2, 0, 0})   // 2m resolution hint, 0 contact overlap, 0 rotation factor. NOLINT
    ->Args({3, 0, 0})   // 3m resolution hint, 0 contact overlap, 0 rotation factor. NOLINT
    ->Args({4, 0, 0})   // 4m resolution hint, 0 contact overlap, 0 rotation factor. NOLINT
    ->Args({2, 1, 0})   // 2m resolution hint, 1 contact overlap, 0 rotation factor. NOLINT
    ->Args({2, 2, 0})   // 2m resolution hint, 2 contact overlap, 0 rotation factor. NOLINT
    ->Args({2, 3, 0})   // 2m resolution hint, 3 contact overlap, 0 rotation factor. NOLINT
    ->Args({2, 4, 0})   // 2m resolution hint, 4 contact overlap, 0 rotation factor. NOLINT
    ->Args({2, 0, 1})   // 2m resolution hint, 0 contact overlap, 1 rotation factor. NOLINT
    ->Args({2, 0, 2})   // 2m resolution hint, 0 contact overlap, 2 rotation factor. NOLINT
    ->Args({2, 0, 3})   // 2m resolution hint, 0 contact overlap, 3 rotation factor. NOLINT
    ->Args({2, 1, 1})   // 2m resolution hint, 1 contact overlap, 1 rotation factor. NOLINT
    ->Args({2, 2, 2});  // 2m resolution hint, 2 contact overlap, 2 rotation factor. NOLINT

void ReportContactSurfaces() {
  std::cout << "Resulting contact surface sizes:" << std::endl;
  for (const auto& output :
       MeshIntersectionBenchmark::contact_surface_result_output) {
    std::cout << fmt::format(" - {}", output) << std::endl;
  }
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake

int main(int argc, char** argv) {
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  drake::geometry::internal::ReportContactSurfaces();
}
