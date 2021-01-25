#include "drake/geometry/proximity/make_capsule_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

// Signed distance to capsule boundary is radius of the capsule minus the
// distance to medial axis of the capsule. This takes the convention that signed
// distance is positive inside the capsule and negative outside.
struct DistanceToCapsuleBoundaryFromPointInside {
  explicit DistanceToCapsuleBoundaryFromPointInside(const Capsule& capsule_in)
      : capsule(capsule_in) {
    p = Vector3d(0, 0, 0.5 * capsule_in.length());
    q = Vector3d(0, 0, -0.5 * capsule_in.length());
  }

  double operator()(const Vector3d& r) const {
    // Find the closest point on the segment pq to r.
    double t = std::clamp((r - p).dot(q - p) / capsule.length(), 0.0, 1.0);
    Vector3d closest = p + t * (q - p);
    return capsule.radius() - sqrt((closest - r).dot(closest - r));
  }

  Capsule capsule;
  Vector3d p;  // Top endpoint of medial axis.
  Vector3d q;  // Bottom endpoint of medial axis.
};

// Returns true if `tetrahedron` in `mesh` has at least one vertex lying on the
// medial axis and at least one vertex lying on the boundary.
//
// We use the given `tolerance` for comparing distances.
//
// @pre No vertex of the tetrahedron is outside the capsule.
//
// Here we omit the frame notations because everything is expressed in the
// frame of the capsule. It might change in the future.
bool IsTetrahedronRespectingMa(const VolumeElement& tetrahedron,
                               const VolumeMesh<double>& mesh,
                               const Capsule& capsule, const double tolerance) {
  DistanceToCapsuleBoundaryFromPointInside distance_to_boundary(capsule);

  int num_boundary = 0;
  int num_medial = 0;

  for (int v = 0; v < mesh.kVertexPerElement; ++v) {
    const Vector3d r_MV = mesh.vertex(tetrahedron.vertex(v)).r_MV();
    const double dist = distance_to_boundary(r_MV);
    if (capsule.radius() - dist < tolerance) {
      num_medial++;
    }
    if (dist < tolerance) {
      num_boundary++;
    }
  }

  return num_boundary > 0 && num_medial > 0;
}

// Verifies that a tetrahedral mesh of a capsule from
// MakeCapsuleVolumeMeshWithMa() satisfies all these properties:
//
//   A. The mesh is conforming.
//   B. The mesh conforms to the capsule.
//   C. The mesh conforms to the capsule's medial axis.
//
// We will use this function in all unit tests of
// MakeCapsuleVolumeMeshWithMa().
//
// Here we omit the frame notations because everything is expressed in the
// frame of the capsule.
void VerifyCapsuleMeshWithMa(const VolumeMesh<double>& mesh,
                             const Capsule& capsule) {
  // A. The mesh is conforming.
  // A1. The mesh has unique vertices.
  const int num_vertices = mesh.num_vertices();
  for (VolumeVertexIndex i(0); i < num_vertices; ++i) {
    for (VolumeVertexIndex j(i + 1); j < num_vertices; ++j) {
      const bool vertex_is_unique =
          mesh.vertex(i).r_MV() != mesh.vertex(j).r_MV();
      ASSERT_TRUE(vertex_is_unique) << "The mesh has duplicated vertices.";
    }
  }
  // A2. Euler characteristic = 1. This is a necessary condition (but may not
  //     be sufficient) for conforming tetrahedra (two tetrahedra intersect in
  //     their shared face, or shared edge, or shared vertex, or not at all.
  //     There is no partial overlapping of two tetrahedra.).
  const int euler_characteristic = ComputeEulerCharacteristic(mesh);
  ASSERT_EQ(1, euler_characteristic)
      << "The mesh's tetrahedra are not conforming because the mesh's Euler "
         "characteristic is "
      << euler_characteristic << " instead of 1.";

  // B. The mesh conforms to the capsule.
  const double tolerance = DistanceToPointRelativeTolerance(
      std::max(capsule.length() / 2., capsule.radius()));
  DistanceToCapsuleBoundaryFromPointInside distance_to_boundary(capsule);
  for (const VolumeVertex<double>& v : mesh.vertices()) {
    ASSERT_TRUE(distance_to_boundary(v.r_MV()) + tolerance >= 0)
        << "A mesh vertex is outside the capsule.";
  }

  // C. The mesh conforms to the capsule's medial axis.
  // C1. No tetrahedron has all four vertices on the capsule's boundary, i.e.,
  //     each tetrahedron has at least one interior vertex.
  std::vector<VolumeVertexIndex> boundary_vertices =
      CollectUniqueVertices(IdentifyBoundaryFaces(mesh.tetrahedra()));
  for (const VolumeElement tetrahedron : mesh.tetrahedra()) {
    bool tetrahedron_has_an_interior_vertex = false;
    for (int i = 0;
         i < mesh.kVertexPerElement && !tetrahedron_has_an_interior_vertex;
         ++i) {
      tetrahedron_has_an_interior_vertex =
          boundary_vertices.end() == find(boundary_vertices.begin(),
                                          boundary_vertices.end(),
                                          tetrahedron.vertex(i));
    }
    ASSERT_TRUE(tetrahedron_has_an_interior_vertex)
        << "A tetrahedron has all its vertices on the boundary.";
  }

  // C2. Each tetrahedron conforms to MA. Assume every mesh vertex is in
  //     the capsule (B1 test above).
  for (const VolumeElement& tetrahedron : mesh.tetrahedra()) {
    bool tetrahedron_conform_to_MA =
        IsTetrahedronRespectingMa(tetrahedron, mesh, capsule, tolerance);
    ASSERT_TRUE(tetrahedron_conform_to_MA)
        << "A tetrahedron does not conform to the medial axis of the "
           "capsule.";
  }
}

GTEST_TEST(MakeCapsuleVolumeMesTest, Long) {
  const double radius = 1.0;
  const double length = 3.0;
  const double resolution_hint = 0.05;
  const Capsule capsule(radius, length);
  VolumeMesh<double> mesh =
      MakeCapsuleVolumeMesh<double>(capsule, resolution_hint);
  VerifyCapsuleMeshWithMa(mesh, capsule);
}

GTEST_TEST(MakeCapsuleVolumeMeshWithMaTest, Medium) {
  const double radius = 1.0;
  const double length = 2.0;
  const double resolution_hint = 0.5;
  const Capsule capsule(radius, length);
  VolumeMesh<double> mesh =
      MakeCapsuleVolumeMesh<double>(capsule, resolution_hint);

  VerifyCapsuleMeshWithMa(mesh, capsule);
}

GTEST_TEST(MakeCapsuleVolumeMeshWithMaTest, Short) {
  const double radius = 1.0;
  const double length = 1.0;
  const double resolution_hint = 0.5;
  const Capsule capsule(radius, length);
  VolumeMesh<double> mesh =
      MakeCapsuleVolumeMesh<double>(capsule, resolution_hint);
  VerifyCapsuleMeshWithMa(mesh, capsule);
}

GTEST_TEST(MakeCapsuleVolumeMeshWithMaTest, Coarsest) {
  const double radius = 1.0;
  const double length = 1.0;
  // Using very large resolution_hint should gives us the coarsest mesh with
  // the number of vertices on each circular rim equal 3.
  const double resolution_hint = 100.0;
  const Capsule capsule(radius, length);
  VolumeMesh<double> mesh =
      MakeCapsuleVolumeMesh<double>(capsule, resolution_hint);
  VerifyCapsuleMeshWithMa(mesh, capsule);
}

GTEST_TEST(MakeCapsuleVolumeMesh, CoarsestMesh) {
  const double radius = 1.0;
  const double length = 2.0;
  // A `resolution_hint` greater than 2/3 π times the radius of the capsule
  // should give the coarsest mesh. We use a scaling factor larger than that
  // to trigger the coarse mesh. It should give a mesh with 15 tetrahedra.
  const double resolution_hint_above = M_PI * radius;
  auto mesh_coarse = MakeCapsuleVolumeMesh<double>(Capsule(radius, length),
                                                   resolution_hint_above);

  EXPECT_EQ(15, mesh_coarse.num_elements());

  // A `resolution_hint` of 2/4 π times the radius of the capsule should
  // give then next coarsest mesh (4 vertices on the circlar rim).
  // The resulting mesh should have 20 tetrahedra.
  const double resolution_hint_below = 0.5 * M_PI * radius;
  auto mesh_finer = MakeCapsuleVolumeMesh<double>(Capsule(radius, length),
                                                  resolution_hint_below);
  EXPECT_EQ(20, mesh_finer.num_elements());
}

// This test verifies that the volume of the tessellated
// capsule converges to the exact value as the tessellation is refined.
GTEST_TEST(MakeCapsuleVolumeMesh, VolumeConvergence) {
  const double height = 2;
  const double radius = 1;
  double resolution_hint = 3.0;  // Hint to coarsest mesh.
  auto mesh0 =
      MakeCapsuleVolumeMesh<double>(Capsule(radius, height), resolution_hint);

  const double volume0 = mesh0.CalcVolume();
  // Volume of the two hemispheres + volume of the cylinder
  const double capsule_volume =
      M_PI * radius * radius * ((4. / 3.) * radius + height);

  // Initial error in the computation of the volume.
  double prev_error = capsule_volume - volume0;

  for (int level = 1; level < 6; ++level) {
    resolution_hint /= 2.0;
    auto mesh = MakeCapsuleVolumeMesh<double>(
        drake::geometry::Capsule(radius, height), resolution_hint);

    // Verify that the volume monotonically converges towards the exact volume
    // of the given capsule.
    const double volume = mesh.CalcVolume();
    const double error = capsule_volume - volume;

    EXPECT_GT(error, 0.0);
    EXPECT_LT(error, prev_error);

    // Always compare against last computed error to show monotonic convergence.
    prev_error = error;
  }
}

// Confirm that the mesh is well formed (i.e., with no duplicate vertices or
// tetrahedra) by computing its Euler characteristic. Looking at the
// tetrahedral mesh as a convex 4 dimensional simplicial complex, this test
// computes the generalized Euler characteristic:
//
// χ = k₀ - k₁ + k₂ - k₃
//
// where kᵢ is the number of i-simplexes in the complex. For a convex mesh that
// is homeomorphic to a 3 dimensional ball, χ = 1.
GTEST_TEST(MakeCapsuleVolumeMesh, EulerCharacteristic) {
  const double height = 2;
  const double radius = 1;

  const int expected_euler_characteristic = 1;

  for (const double resolution_hint : {2., 1., 0.5, 0.25, 0.125, 0.0625}) {
    auto mesh = MakeCapsuleVolumeMesh<double>(
        drake::geometry::Capsule(radius, height), resolution_hint);

    EXPECT_EQ(ComputeEulerCharacteristic(mesh), expected_euler_characteristic);
  }
}

// Smoke test only. Assume correctness of MakeCapsuleVolumeMesh() and
// ConvertVolumeToSurfaceMesh(). The resolution_hint 3 times the
// radius of the capsule produces a triangular prism and two tetrahedral caps
// with 12 triangles and 8 vertices, which is the coarsest surface mesh that our
// algorithm can produce.
GTEST_TEST(MakeCapsuleSurfaceMesh, GenerateSurface) {
  const double radius = 1.0;
  const double length = 2.0;
  const double resolution_hint = 3.0;
  const Capsule capsule(radius, length);
  SurfaceMesh<double> surface_mesh =
      MakeCapsuleSurfaceMesh<double>(capsule, resolution_hint);
  EXPECT_EQ(surface_mesh.num_faces(), 12);
  EXPECT_EQ(surface_mesh.num_vertices(), 8);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
