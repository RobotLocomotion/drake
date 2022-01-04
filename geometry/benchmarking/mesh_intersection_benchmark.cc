#include "fmt/format.h"
#include <benchmark/benchmark.h>

#include "drake/geometry/proximity/make_ellipsoid_field.h"
#include "drake/geometry/proximity/make_ellipsoid_mesh.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/mesh_intersection.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* @defgroup mesh_intersection_benchmarks Mesh Intersection Benchmarks
 @ingroup proximity_queries

 The benchmark evaluates mesh intersection between compliant and rigid meshes.

 It computes the contact surface formed from the intersection of an ellipsoid
 and a sphere using broad-phase culling (via a bounding volume hierarchy).
 Arguments include:
 - __resolution__: An enumeration in the integer range from 0 to 3 that guides
   the level of mesh refinement, where 0 produces the coarsest meshes and 3
   produces the finest meshes.
 - __contact overlap__: An enumeration in the integer range from 0 to 4 that
   correlates with the size of the resultant contact surface between the two
   meshes, where 0 produces the least contact and 4 produces the most contact.
   For the final accurate size of the contact surface, refer to the report at
   the bottom of the benchmark output.
 - __rotation factor__: An enumeration in the integer range from 0 to 3 that
   affects how much the meshes are in axis alignment, where 0 is aligned and
   3 is maximally misaligned. The given scalar is used as a multiplicative
   factor of PI/4 and applied such that the sphere is rotated relative to the
   ellipsoid.

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

 Note that the terminating line numbers [xx] have been added for referencing in
 discussions below, but otherwise the output will be something akin to:

 ```
Run on (56 X 3500 MHz CPU s)
CPU Caches:
  L1 Data 32K (x28)
  L1 Instruction 32K (x28)
  L2 Unified 256K (x28)
  L3 Unified 35840K (x2)
Load Average: 21.52, 41.04, 26.99
----------------------------------------------------------------------------------------------------                // NOLINT(*)
Benchmark                                                          Time             CPU   Iterations   Line Number  // NOLINT(*)
----------------------------------------------------------------------------------------------------                // NOLINT(*)
MeshIntersectionBenchmark/RigidSoftMesh/0/4/0/min_time:2.000      0.629 ms        0.629 ms         4408   [14]         // NOLINT(*)
MeshIntersectionBenchmark/RigidSoftMesh/1/4/0/min_time:2.000       2.15 ms         2.15 ms         1328   [15]         // NOLINT(*)
MeshIntersectionBenchmark/RigidSoftMesh/2/4/0/min_time:2.000       3.14 ms         3.14 ms          893   [16]         // NOLINT(*)
MeshIntersectionBenchmark/RigidSoftMesh/3/4/0/min_time:2.000       14.3 ms         14.3 ms          192   [17]         // NOLINT(*)
MeshIntersectionBenchmark/RigidSoftMesh/2/0/0/min_time:2.000      0.000 ms        0.000 ms     54932081   [18]         // NOLINT(*)
MeshIntersectionBenchmark/RigidSoftMesh/2/1/0/min_time:2.000      0.023 ms        0.023 ms       119450   [19]         // NOLINT(*)
MeshIntersectionBenchmark/RigidSoftMesh/2/2/0/min_time:2.000      0.119 ms        0.119 ms        23142   [20]         // NOLINT(*)
MeshIntersectionBenchmark/RigidSoftMesh/2/3/0/min_time:2.000       1.87 ms         1.87 ms         1486   [21]         // NOLINT(*)
MeshIntersectionBenchmark/RigidSoftMesh/2/4/1/min_time:2.000       2.95 ms         2.95 ms          944   [22]         // NOLINT(*)
MeshIntersectionBenchmark/RigidSoftMesh/2/4/2/min_time:2.000       3.15 ms         3.15 ms          894   [23]         // NOLINT(*)
MeshIntersectionBenchmark/RigidSoftMesh/2/4/3/min_time:2.000       3.21 ms         3.21 ms          871   [24]         // NOLINT(*)
MeshIntersectionBenchmark/RigidSoftMesh/2/3/1/min_time:2.000       1.94 ms         1.94 ms         1448   [25]         // NOLINT(*)
MeshIntersectionBenchmark/RigidSoftMesh/2/2/2/min_time:2.000      0.127 ms        0.127 ms        22007   [26]         // NOLINT(*)
Resulting contact surface sizes:
 - RigidSoftMesh/0/4/0: 93.76 m^2, 448 triangles
 - RigidSoftMesh/1/4/0: 93.76 m^2, 1592 triangles
 - RigidSoftMesh/2/4/0: 107.59 m^2, 2848 triangles
 - RigidSoftMesh/3/4/0: 111.67 m^2, 14336 triangles
 - RigidSoftMesh/2/0/0: 0.00 m^2, 0 triangles
 - RigidSoftMesh/2/1/0: 0.00 m^2, 0 triangles
 - RigidSoftMesh/2/2/0: 1.50 m^2, 58 triangles
 - RigidSoftMesh/2/3/0: 48.21 m^2, 2226 triangles
 - RigidSoftMesh/2/4/1: 106.41 m^2, 3804 triangles
 - RigidSoftMesh/2/4/2: 106.89 m^2, 3848 triangles
 - RigidSoftMesh/2/4/3: 107.44 m^2, 3808 triangles
 - RigidSoftMesh/2/3/1: 48.26 m^2, 2190 triangles
 - RigidSoftMesh/2/2/2: 1.52 m^2, 62 triangles

 ```

 <h4>Interpreting the benchmark</h4>

 Each line in the table represents a particular configuration of the benchmark
 parameters of the form:

 ```
 MeshIntersectionBenchmark/TestName/resolution/contact_overlap/rotation_factor/min_time
 ```

   - __TestName__: RigidSoftMesh
   - __resolution__: Affects the resolution of the ellipsoid and sphere
     meshes. Valid values must be one of [0, 1, 2, 3], where 0 produces the
     coarsest meshes and 3 produces the finest meshes. This is converted behind
     the scenes to a resolution hint, see AddRigidHydroelasticProperties() for
     more details.
   - __contact_overlap__: Affects the size of the resulting contact surface by
     translating the rigid mesh relative to the compliant mesh. Contact overlap
     should be one of the following enumeration values representing:
     - 0: No contact and no overlapping bounding volumes at all.
     - 1: No contact but overlapping bounding volumes.
     - 2: The minimal, or at least very small, contact surface.
     - 3: An intermediate sized contact surface.
     - 4: The maximal contact surface (in area).
   - __rotation_factor__: How much rotation to offset between the compliant and
     the rigid mesh to increase axis misalignment. Given an offset, int `i`, the
     resulting rotation is calculated as `i / max_factor * π/4` radians around
     the geometry's shortest axis, in this case, the x-axis. When rotation
     factor is zero, they are perfectly aligned, as rotation factor increases,
     the dot product between the two sets of misaligned axes are minimized. We
     set the max_factor to 3, so valid rotation factors must be one of
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
 */

using Eigen::AngleAxis;
using Eigen::Vector3d;
using math::RigidTransformd;

const double kElasticModulus = 1.0e5;
const double kMaxRotationFactor = 3.;
const double kSphereDimension = 3.;
const Vector3d kEllipsoidDimension{3.01, 3.5, 4.};
const double kResolutionHint[4] = {4., 3., 2., 1.};
const Vector3d kContactOverlapTranslation[5] = {
    Vector3d{7, 7, 7},        // 0: No overlap at all.
    Vector3d{4, 4, 4},        // 1: Overlapping bounding volumes.
    Vector3d{3.5, 3.5, 3.5},  // 2: Minimal contact surface.
    Vector3d{1.2, 1.2, 1.2},  // 3: Intermediate sized contact surface.
    Vector3d{0, 0, 0}};       // 4: Maximal contact surface.

class MeshIntersectionBenchmark : public benchmark::Fixture {
 public:
  MeshIntersectionBenchmark()
      : ellipsoid_{kEllipsoidDimension[0], kEllipsoidDimension[1],
                   kEllipsoidDimension[2]},
        sphere_{kSphereDimension},
        mesh_S_(MakeEllipsoidVolumeMesh<double>(
            ellipsoid_, 1, TessellationStrategy::kDenseInteriorVertices)),
        field_S_(MakeEllipsoidPressureField<double>(ellipsoid_, &mesh_S_,
                                                    kElasticModulus)),
        mesh_R_(MakeEllipsoidSurfaceMesh<double>(ellipsoid_, 1)) {}

  /* Parse arguments from the benchmark state.
  @return A tuple representing the resolution, the contact overlap, and the
          rotation factor.  */
  static std::tuple<int, int, int> ReadState(
      const benchmark::State& state) {
    return std::make_tuple(state.range(0), state.range(1), state.range(2));
  }

  /* Set up the two ellipsoid meshes and their relative transform.  */
  void SetupMeshes(const benchmark::State& state) {
    const auto [resolution, contact_overlap, rotation_factor] =
        ReadState(state);
    const double resolution_hint = kResolutionHint[resolution];
    mesh_S_ = MakeEllipsoidVolumeMesh<double>(
        ellipsoid_, resolution_hint,
        TessellationStrategy::kDenseInteriorVertices);
    field_S_ = MakeEllipsoidPressureField<double>(ellipsoid_, &mesh_S_,
                                                  kElasticModulus);
    mesh_R_ = MakeSphereSurfaceMesh<double>(sphere_, resolution_hint);
    X_SR_ = RigidTransformd{
        AngleAxis(rotation_factor / kMaxRotationFactor * M_PI / 4,
                  Vector3d{1, 1, 1}.normalized()),
        kContactOverlapTranslation[contact_overlap]};
  }

  /* Record metrics on the resulting contact surface for reporting later.  */
  void RecordContactSurfaceResult(const TriangleSurfaceMesh<double>* surface_SR,
                                  const std::string& test_name,
                                  const benchmark::State& state) {
    const int num_elements =
        surface_SR == nullptr ? 0 : surface_SR->num_elements();
    const double area = surface_SR == nullptr ? 0 : surface_SR->total_area();
    const auto [resolution, contact_overlap, rotation_factor] =
        ReadState(state);
    const std::string result_key = fmt::format(
        "{}/{}/{}/{}", test_name, resolution, contact_overlap, rotation_factor);
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
  Sphere sphere_;
  VolumeMesh<double> mesh_S_;
  VolumeMeshFieldLinear<double, double> field_S_;
  TriangleSurfaceMesh<double> mesh_R_;
  RigidTransformd X_SR_;
};
std::set<std::string> MeshIntersectionBenchmark::contact_surface_result_keys;
std::vector<std::string>
    MeshIntersectionBenchmark::contact_surface_result_output;

BENCHMARK_DEFINE_F(MeshIntersectionBenchmark, RigidSoftMesh)
// NOLINTNEXTLINE(runtime/references)
(benchmark::State& state) {
  SetupMeshes(state);
  const auto bvh_S = Bvh<Obb, VolumeMesh<double>>(mesh_S_);
  const auto bvh_R = Bvh<Obb, TriangleSurfaceMesh<double>>(mesh_R_);
  std::unique_ptr<TriangleSurfaceMesh<double>> surface_SR;
  std::unique_ptr<TriangleSurfaceMeshFieldLinear<double, double>> e_SR;
  for (auto _ : state) {
    SurfaceVolumeIntersector<TriangleSurfaceMesh<double>> intersector;
    intersector.SampleVolumeFieldOnSurface(field_S_, bvh_S, mesh_R_, bvh_R,
                                           X_SR_, TriMeshBuilder<double>());
    surface_SR = intersector.release_mesh();
    e_SR = intersector.release_field();
  }
  RecordContactSurfaceResult(surface_SR.get(), "RigidSoftMesh", state);
}
BENCHMARK_REGISTER_F(MeshIntersectionBenchmark, RigidSoftMesh)
    ->Unit(benchmark::kMillisecond)
    ->MinTime(2)
    ->Args({0, 4, 0})   // 0 resolution, 4 contact overlap, 0 rotation factor.
    ->Args({1, 4, 0})   // 1 resolution, 4 contact overlap, 0 rotation factor.
    ->Args({2, 4, 0})   // 2 resolution, 4 contact overlap, 0 rotation factor.
    ->Args({3, 4, 0})   // 3 resolution, 4 contact overlap, 0 rotation factor.
    ->Args({2, 0, 0})   // 2 resolution, 0 contact overlap, 0 rotation factor.
    ->Args({2, 1, 0})   // 2 resolution, 1 contact overlap, 0 rotation factor.
    ->Args({2, 2, 0})   // 2 resolution, 2 contact overlap, 0 rotation factor.
    ->Args({2, 3, 0})   // 2 resolution, 3 contact overlap, 0 rotation factor.
    ->Args({2, 4, 1})   // 2 resolution, 4 contact overlap, 1 rotation factor.
    ->Args({2, 4, 2})   // 2 resolution, 4 contact overlap, 2 rotation factor.
    ->Args({2, 4, 3})   // 2 resolution, 4 contact overlap, 3 rotation factor.
    ->Args({2, 3, 1})   // 2 resolution, 3 contact overlap, 1 rotation factor.
    ->Args({2, 2, 2});  // 2 resolution, 2 contact overlap, 2 rotation factor.

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
