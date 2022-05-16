#include "drake/geometry/proximity/make_capsule_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

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
  int num_boundary = 0;
  int num_medial = 0;

  for (int v = 0; v < mesh.kVertexPerElement; ++v) {
    const Vector3d r_MV = mesh.vertex(tetrahedron.vertex(v));
    const double dist = CalcDistanceToSurface(capsule, r_MV);
    if (capsule.radius() + dist < tolerance) {
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
  for (int i = 0; i < num_vertices; ++i) {
    for (int j = i + 1; j < num_vertices; ++j) {
      const bool vertex_is_unique = mesh.vertex(i) != mesh.vertex(j);
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
  for (const Vector3d& v : mesh.vertices()) {
    ASSERT_TRUE(CalcDistanceToSurface(capsule, v) - tolerance <= 0)
        << "A mesh vertex is outside the capsule.";
  }

  // C. The mesh conforms to the capsule's medial axis.
  // C1. No tetrahedron has all four vertices on the capsule's boundary, i.e.,
  //     each tetrahedron has at least one interior vertex.
  const std::vector<int> boundary_vertices =
      CollectUniqueVertices(IdentifyBoundaryFaces(mesh.tetrahedra()));
  for (const VolumeElement& tetrahedron : mesh.tetrahedra()) {
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
    const bool tetrahedron_conform_to_MA =
        IsTetrahedronRespectingMa(tetrahedron, mesh, capsule, tolerance);
    ASSERT_TRUE(tetrahedron_conform_to_MA)
        << "A tetrahedron does not conform to the medial axis of the "
           "capsule.";
  }
}

GTEST_TEST(MakeCapsuleVolumeMesTest, Long) {
  const double radius = 1.0;
  const double length = 3.0;
  const double resolution_hint = 0.5;
  const Capsule capsule(radius, length);
  const VolumeMesh<double> mesh =
      MakeCapsuleVolumeMesh<double>(capsule, resolution_hint);
  VerifyCapsuleMeshWithMa(mesh, capsule);
}

GTEST_TEST(MakeCapsuleVolumeMeshWithMaTest, Medium) {
  const double radius = 1.0;
  const double length = 2.0;
  const double resolution_hint = 0.5;
  const Capsule capsule(radius, length);
  const VolumeMesh<double> mesh =
      MakeCapsuleVolumeMesh<double>(capsule, resolution_hint);

  VerifyCapsuleMeshWithMa(mesh, capsule);
}

GTEST_TEST(MakeCapsuleVolumeMeshWithMaTest, Short) {
  const double radius = 1.0;
  const double length = 0.5;
  const double resolution_hint = 0.5;
  const Capsule capsule(radius, length);
  const VolumeMesh<double> mesh =
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
  const VolumeMesh<double> mesh =
      MakeCapsuleVolumeMesh<double>(capsule, resolution_hint);
  VerifyCapsuleMeshWithMa(mesh, capsule);
}

GTEST_TEST(MakeCapsuleVolumeMesh, CoarsestMesh) {
  const double radius = 1.0;
  const double length = 2.0;
  const int n = 3;  // number of verts per circle.
  // A `resolution_hint` equal to 2/3·π times the radius of the
  // capsule should give the coarsest mesh.
  const double resolution_hint_above = (2. / n) * M_PI * radius;
  const VolumeMesh<double> mesh_coarse = MakeCapsuleVolumeMesh<double>(
      Capsule(radius, length), resolution_hint_above);
  // V = 2n⌊n/2⌋ + 4
  EXPECT_EQ(2 * n * (n / 2) + 4, mesh_coarse.num_vertices());
  // F = 4n(⌊n/2⌋ - 1) + 5n
  EXPECT_EQ(4 * n * ((n / 2) - 1) + 5 * n, mesh_coarse.num_elements());
}

GTEST_TEST(MakeCapsuleVolumeMesh, CoarsestMeshClamped) {
  const double radius = 1.0;
  const double length = 2.0;
  const int n = 3;  // number of verts per circle.
  // A `resolution_hint` greater than  2/3·π times the radius of the
  // capsule should be clamped and also give the coarsest mesh.
  const double resolution_hint_above = M_PI * radius;
  const VolumeMesh<double> mesh_coarse = MakeCapsuleVolumeMesh<double>(
      Capsule(radius, length), resolution_hint_above);
  // V = 2n⌊n/2⌋ + 4
  EXPECT_EQ(2 * n * (n / 2) + 4, mesh_coarse.num_vertices());
  // F = 4n(⌊n/2⌋ - 1) + 5n
  EXPECT_EQ(4 * n * ((n / 2) - 1) + 5 * n, mesh_coarse.num_elements());
}

GTEST_TEST(MakeCapsuleVolumeMesh, SecondCoarsestMesh) {
  const double radius = 1.0;
  const double length = 2.0;
  const int n = 4;  // number of verts per circle.
  // A `resolution_hint` of 2/4·π times the radius of the capsule should
  // give the next coarsest mesh (4 vertices on the circular rim).
  const double resolution_hint_below = (2. / n) * M_PI * radius;
  const VolumeMesh<double> mesh_finer = MakeCapsuleVolumeMesh<double>(
      Capsule(radius, length), resolution_hint_below);
  // V = 2n⌊n/2⌋ + 4
  EXPECT_EQ(2 * n * (n / 2) + 4, mesh_finer.num_vertices());
  // F = 4n(⌊n/2⌋ - 1) + 5n
  EXPECT_EQ(4 * n * ((n / 2) - 1) + 5 * n, mesh_finer.num_elements());
}

GTEST_TEST(MakeCapsuleVolumeMesh, FinestMesh) {
  const double radius = 1.0;
  const double length = 2.0;
  const int n = 706;  // number of verts per circle.
  // A `resolution_hint` of 2/706·π times the radius of the capsule should
  // give the finest possible mesh (706 vertices on the circular rim).
  const double resolution_hint = 2. * M_PI * radius / n;
  const VolumeMesh<double> mesh_finest =
      MakeCapsuleVolumeMesh<double>(Capsule(radius, length), resolution_hint);
  // V = 2n⌊n/2⌋ + 4
  EXPECT_EQ(2 * n * (n / 2) + 4, mesh_finest.num_vertices());
  // F = 4n(⌊n/2⌋ - 1) + 5n
  EXPECT_EQ(4 * n * ((n / 2) - 1) + 5 * n, mesh_finest.num_elements());
}

GTEST_TEST(MakeCapsuleVolumeMesh, FinestMeshClamped) {
  const double radius = 1.0;
  const double length = 2.0;
  const int n = 706;  // number of verts per circle.
  // A `resolution_hint` smaller than 2/706·π times the radius of the capsule
  // should be clamped and just give the finest possible mesh (706 vertices on
  // the circular rim). Here we give a hint twice as small as the smallest
  // valid hint.
  const double resolution_hint = M_PI * radius / n;
  const VolumeMesh<double> mesh_finest =
      MakeCapsuleVolumeMesh<double>(Capsule(radius, length), resolution_hint);
  // V = 2n⌊n/2⌋ + 4
  EXPECT_EQ(2 * n * (n / 2) + 4, mesh_finest.num_vertices());
  // F = 4n(⌊n/2⌋ - 1) + 5n
  EXPECT_EQ(4 * n * ((n / 2) - 1) + 5 * n, mesh_finest.num_elements());
}

// This test verifies that the error between the volume of the tessellated
// capsule and the true volume of the parameterized capsule monotonically
// decreases as the tesselation is refined.
GTEST_TEST(MakeCapsuleVolumeMesh, VolumeConvergence) {
  const double height = 2;
  const double radius = 1;
  double resolution_hint = 3.0;  // Hint to coarsest mesh.
  const VolumeMesh<double> mesh0 =
      MakeCapsuleVolumeMesh<double>(Capsule(radius, height), resolution_hint);

  const double volume0 = mesh0.CalcVolume();
  // Volume of the two hemispheres + volume of the cylinder
  const double capsule_volume =
      M_PI * radius * radius * ((4. / 3.) * radius + height);

  // Initial error in the computation of the volume.
  double prev_error = capsule_volume - volume0;

  for (int level = 1; level < 6; ++level) {
    resolution_hint /= 2.0;
    const VolumeMesh<double> mesh = MakeCapsuleVolumeMesh<double>(
        drake::geometry::Capsule(radius, height), resolution_hint);

    // Verify that the volume monotonically decreases towards the exact volume
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

  // Choose values that will trigger distinct tesselations of the capsule.
  for (const int n : {3, 4, 5, 6, 7, 8}) {
    double resolution_hint = M_PI * radius / n;

    const VolumeMesh<double> mesh = MakeCapsuleVolumeMesh<double>(
        drake::geometry::Capsule(radius, height), resolution_hint);

    EXPECT_EQ(ComputeEulerCharacteristic(mesh), expected_euler_characteristic);
  }
}

// Smoke test only. MakeCapsuleSurfaceMesh consists only of a single call to
// MakeCapsuleVolumeMesh() and a single call to ConvertVolumeToSurfaceMesh(),
// assuming the correctness of both of those functions. The resolution_hint
// 2/3·π·radius produces a triangular prism and two tetrahedral caps with 12
// triangles and 8 vertices, which is the coarsest surface mesh that our
// algorithm can produce.
GTEST_TEST(MakeCapsuleSurfaceMesh, GenerateSurface) {
  const double radius = 1.0;
  const double length = 2.0;
  const double resolution_hint = 2.0 * M_PI * radius / 3;
  const Capsule capsule(radius, length);
  const TriangleSurfaceMesh<double> surface_mesh =
      MakeCapsuleSurfaceMesh<double>(capsule, resolution_hint);
  EXPECT_EQ(surface_mesh.num_triangles(), 12);
  EXPECT_EQ(surface_mesh.num_vertices(), 8);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
