#include "drake/geometry/proximity/make_cylinder_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

// This functor assumes the query point is on or inside the cylinder. Passing a
// point outside the cylinder to this functor might get the wrong value of
// the distance-to-boundary function.
//
// To illustrate the issue of an outside point, consider a cylinder of radius
// 1 and length 2, whose 2D cross section forms the square [-1,1]x[-1,1] in
// the X-Z plane of the cylinder as shown in the following picture. The
// outside point (2,0,2) should have the value √2 for the distance to the
// boundary of this cylinder. However, this functor would return -1
// (radial_distance=-1, top_distance=-1, bottom_distance=+3) for this
// outside point.
//
//             +Z (axis of rotation)
//              ↑
//              |    ┊    ●(2,0,2)
//              |    ┊
//         ┏━━━+1━━━━┓┈┈┈┈┈
//         ┃    ┃    ┃
//        -1━━━━0━━━+1 -----→ +X
//         ┃    ┃    ┃
//         ┗━━━-1━━━━┛┈┈┈┈┈
//
struct DistanceToCylinderBoundaryFromPointInside {
  explicit DistanceToCylinderBoundaryFromPointInside(
      const Cylinder& cylinder_in)
      : cylinder(cylinder_in) {}

  double operator()(const Vector3d& point_in_cylinder) const {
    const double x = point_in_cylinder.x();
    const double y = point_in_cylinder.y();
    const double z = point_in_cylinder.z();
    const double radial_distance = cylinder.radius() - sqrt(x * x + y * y);
    const double top_z = cylinder.length() / 2;
    const double bottom_z = -top_z;
    const double bottom_distance = z - bottom_z;
    const double top_distance = top_z - z;
    return std::min({radial_distance, bottom_distance, top_distance});
  }

  Cylinder cylinder;
};

// Returns true if `tetrahedron` in `mesh` has its four vertices belonging to
// the same block of the medial-axis subdivision of `cylinder`, i.e., there is a
// cylinder's face to which all four vertices are _closest_. A face of a
// cylinder is either the bottom circular cap, the top circular cap, or the
// side cylindrical surface. Here a vertex is _closest_ to a cylinder's face
// when there is no other closer face. Notice that a vertex may have multiple
// closest faces, i.e., it is on the medial axis.
//
// We use the given `tolerance` for comparing distances.
//
// @pre No vertex of the tetrahedron is outside the cylinder.
//
// Here we omit the frame notations because everything is expressed in the
// frame of the cylinder. It might change in the future.
bool IsTetrahedronRespectingMa(const VolumeElement& tetrahedron,
                               const VolumeMesh<double>& mesh,
                               const Cylinder& cylinder,
                               const double tolerance) {
  // We define the faces of a cylinder in the following way.
  // Face 0 is the bottom circular cap of the cylinder.
  // Face 1 is the top circular cap of the cylinder.
  // Face 2 is the side cylindrical surface.
  // We require a face index `f` to be 0, 1, or 2 < kNumCylinderFaces = 3.
  const int kNumCylinderFaces = 3;
  // This lambda function assumes the point is on or inside the cylinder.
  // Passing a point outside the cylinder to this function might gets the
  // wrong value of distance to boundary face.
  auto distance_to_boundary_face =
      [&cylinder](int f, const Vector3d& point_in_cylinder) -> double {
    const double x = point_in_cylinder.x();
    const double y = point_in_cylinder.y();
    const double z = point_in_cylinder.z();
    const double top_z = cylinder.length() / 2;
    const double bottom_z = -top_z;
    switch (f) {
      case 0:
        return z - bottom_z;
      case 1:
        return top_z - z;
      default:
        DRAKE_DEMAND(f == 2);
        return cylinder.radius() - sqrt(x * x + y * y);
    }
  };

  DistanceToCylinderBoundaryFromPointInside distance_to_boundary(cylinder);

  // TODO(DamrongGuoy): When PR #14279 (Migrates IsTetrahedronRespectingMa() to
  //  proximity_utilities package) lands, use the shared code instead of the
  //  following code.

  // Each vertex may have multiple closest faces when the vertex is on the
  // medial axis.
  std::vector<int> closest_faces[mesh.kVertexPerElement];
  for (int v = 0; v < mesh.kVertexPerElement; ++v) {
    const Vector3d r_MV = mesh.vertex(tetrahedron.vertex(v));
    const double dist = distance_to_boundary(r_MV);
    for (int f = 0; f < kNumCylinderFaces; ++f) {
      if (distance_to_boundary_face(f, r_MV) - dist <= tolerance) {
        closest_faces[v].push_back(f);
      }
    }
  }
  // Check that at least one face is closest.
  for (int f = 0; f < kNumCylinderFaces; ++f) {
    bool closest_to_this_face = true;
    for (int v = 0; v < mesh.kVertexPerElement; ++v) {
      closest_to_this_face &=
          closest_faces[v].end() !=
          std::find(closest_faces[v].begin(), closest_faces[v].end(), f);
    }
    if (closest_to_this_face) return true;
  }
  return false;
}

// Verifies that a tetrahedral mesh of a cylinder from
// MakeCylinderVolumeMeshWithMa() satisfies all these properties:
//
//   A. The mesh is conforming.
//   B. The mesh conforms to the cylinder.
//   C. The mesh conforms to the cylinder's medial axis.
//
// We will use this function in all unit tests of
// MakeCylinderVolumeMeshWithMa().
//
// Here we omit the frame notations because everything is expressed in the
// frame of the cylinder.
void VerifyCylinderMeshWithMa(const VolumeMesh<double>& mesh,
                              const Cylinder& cylinder,
                              const int num_vertex_per_circle) {
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

  // B. The mesh conforms to the cylinder.
  // B1. Every mesh vertex is in the cylinder.
  const double half_length = cylinder.length() / 2.;
  const double squared_radius = cylinder.radius() * cylinder.radius();
  const double distance_tolerance = DistanceToPointRelativeTolerance(
      std::max(half_length, cylinder.radius()));
  for (const Vector3d& v : mesh.vertices()) {
    const double x = v.x();
    const double y = v.y();
    const double z = v.z();
    bool is_inside_or_on_boundary =
        abs(z) < half_length + distance_tolerance &&
        x * x + y * y < squared_radius + distance_tolerance;
    ASSERT_TRUE(is_inside_or_on_boundary)
        << "A mesh vertex is outside the cylinder.";
  }
  // B2. The volume of the mesh approximates the volume of the discretized
  // cylinder.
  const double mesh_volume = mesh.CalcVolume();
  const double expect_volume = half_length * squared_radius *
                               num_vertex_per_circle *
                               sin(2. * M_PI / num_vertex_per_circle);
  // This is a heuristic tolerance.
  const double volume_tolerance = 10. * distance_tolerance;
  ASSERT_NEAR(mesh_volume, expect_volume, volume_tolerance)
      << "The mesh's volume does not approximate the cylinder's volume enough.";

  // C. The mesh conforms to the cylinder's medial axis.
  // C1. No tetrahedron has all four vertices on the cylinder's boundary, i.e.,
  //     each tetrahedron has at least one interior vertex.
  std::vector<int> boundary_vertices =
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
  // C2. No tetrahedron has all four vertices at the same distance to the
  //     cylinder's surface. Assume every mesh vertex is in the cylinder (B1
  //     test above).
  DistanceToCylinderBoundaryFromPointInside distance_to_boundary(cylinder);
  for (const VolumeElement& tetrahedron : mesh.tetrahedra()) {
    const double distance_v0 =
        distance_to_boundary(mesh.vertex(tetrahedron.vertex(0)));
    bool different_distance_from_v0 = false;
    for (int i = 1; i < mesh.kVertexPerElement && !different_distance_from_v0;
         ++i) {
      different_distance_from_v0 =
          distance_tolerance <
          abs(distance_v0 -
              distance_to_boundary(mesh.vertex(tetrahedron.vertex(i))));
    }
    ASSERT_TRUE(different_distance_from_v0)
        << "A tetrahedron has all vertices at the same distances to"
           " the cylinder's surface.";
  }
  // C3. Each tetrahedron conforms to MA. Assume every mesh vertex is in
  //     the cylinder (B1 test above).
  for (const VolumeElement& tetrahedron : mesh.tetrahedra()) {
    bool tetrahedron_conform_to_MA = IsTetrahedronRespectingMa(
        tetrahedron, mesh, cylinder, distance_tolerance);
    ASSERT_TRUE(tetrahedron_conform_to_MA)
        << "A tetrahedron does not conform to the medial axis of the "
           "cylinder.";
  }
}

GTEST_TEST(MakeCylinderVolumeMeshWithMaTest, Long) {
  const double radius = 1.0;
  const double length = 3.0;
  const double resolution_hint = 0.05;
  const Cylinder cylinder(radius, length);
  VolumeMesh<double> mesh =
      MakeCylinderVolumeMeshWithMa<double>(cylinder, resolution_hint);

  const int num_vertex_per_circle = std::max(
      3,
      static_cast<int>(ceil(2. * M_PI * cylinder.radius() / resolution_hint)));
  EXPECT_EQ(mesh.num_elements(), 5 * num_vertex_per_circle);
  EXPECT_EQ(mesh.num_vertices(), 2 * num_vertex_per_circle + 4);
  VerifyCylinderMeshWithMa(mesh, cylinder, num_vertex_per_circle);
}

GTEST_TEST(MakeCylinderVolumeMeshWithMaTest, Medium) {
  const double radius = 1.0;
  const double length = 2.0;
  const double resolution_hint = 0.5;
  const Cylinder cylinder(radius, length);
  VolumeMesh<double> mesh =
      MakeCylinderVolumeMeshWithMa<double>(cylinder, resolution_hint);

  const int num_vertex_per_circle = std::max(
      3,
      static_cast<int>(ceil(2. * M_PI * cylinder.radius() / resolution_hint)));
  EXPECT_EQ(mesh.num_elements(), 4 * num_vertex_per_circle);
  EXPECT_EQ(mesh.num_vertices(), 2 * num_vertex_per_circle + 3);
  VerifyCylinderMeshWithMa(mesh, cylinder, num_vertex_per_circle);
}

GTEST_TEST(MakeCylinderVolumeMeshWithMaTest, Short) {
  const double radius = 1.0;
  const double length = 1.0;
  const double resolution_hint = 0.5;
  const Cylinder cylinder(radius, length);
  VolumeMesh<double> mesh =
      MakeCylinderVolumeMeshWithMa<double>(cylinder, resolution_hint);

  const int num_vertex_per_circle = std::max(
      3,
      static_cast<int>(ceil(2. * M_PI * cylinder.radius() / resolution_hint)));
  EXPECT_EQ(mesh.num_elements(), 9 * num_vertex_per_circle);
  EXPECT_EQ(mesh.num_vertices(), 3 * num_vertex_per_circle + 3);
  VerifyCylinderMeshWithMa(mesh, cylinder, num_vertex_per_circle);
}

GTEST_TEST(MakeCylinderVolumeMeshWithMaTest, Coarsest) {
  const double radius = 1.0;
  const double length = 1.0;
  // Using very large resolution_hint should gives us the coarsest mesh with
  // the number of vertices on each circular rim equal 3.
  const double resolution_hint = 100.0;
  const Cylinder cylinder(radius, length);
  VolumeMesh<double> mesh =
      MakeCylinderVolumeMeshWithMa<double>(cylinder, resolution_hint);

  const int num_vertex_per_circle = 3;
  EXPECT_EQ(mesh.num_elements(), 9 * num_vertex_per_circle);
  EXPECT_EQ(mesh.num_vertices(), 3 * num_vertex_per_circle + 3);
  VerifyCylinderMeshWithMa(mesh, cylinder, num_vertex_per_circle);
}


GTEST_TEST(MakeCylinderVolumeMesh, CoarsestMesh) {
  const double radius = 1.0;
  const double length = 2.0;
  // A `resolution_hint` greater than √2 times the radius of the cylinder
  // should give the coarsest mesh. We use a scaling factor larger than
  // √2 = 1.41421356... in the sixth decimal digit. It should give a mesh of
  // rectangular prism with 24 tetrahedra.
  const double resolution_hint_above = 1.414214 * radius;
  auto mesh_coarse = MakeCylinderVolumeMesh<double>(Cylinder(radius, length),
                                                    resolution_hint_above);
  EXPECT_EQ(24, mesh_coarse.num_elements());

  // A `resolution_hint` slightly below √2 times the radius of the cylinder
  // should give the next refined mesh.  We use a scaling factor smaller than
  // √2 = 1.41421356... in the sixth decimal digit. It should give a mesh of
  // rectangular prism with 8 * 24 = 192 tetrahedra.
  const double resolution_hint_below = 1.414213 * radius;
  auto mesh_fine = MakeCylinderVolumeMesh<double>(Cylinder(radius, length),
                                                  resolution_hint_below);
  EXPECT_EQ(192, mesh_fine.num_elements());
}

// This test verifies that the volume of the tessellated
// cylinder converges to the exact value as the tessellation is refined.
GTEST_TEST(MakeCylinderVolumeMesh, VolumeConvergence) {
  const double kTolerance = 10.0 * std::numeric_limits<double>::epsilon();

  const double height = 2;
  const double radius = 1;
  double resolution_hint = 2.0;  // Hint to coarsest mesh.
  auto mesh0 =
      MakeCylinderVolumeMesh<double>(Cylinder(radius, height), resolution_hint);

  const double volume0 = mesh0.CalcVolume();
  const double cylinder_volume = height * radius * radius * M_PI;

  // Initial error in the computation of the volume.
  double prev_error = cylinder_volume - volume0;

  // The volume of a rectangular prism with base
  // of size l x w and height h is V = lwh.
  // For our level zero cylinder, we have a prism of height 2 and
  // l = w = sqrt(2) and height = 2.0. Thus its volume is 4.
  const double expected_volume_0 = 4.0;

  EXPECT_NEAR(volume0, expected_volume_0, kTolerance);

  for (int level = 1; level < 6; ++level) {
    resolution_hint /= 2.0;
    auto mesh = MakeCylinderVolumeMesh<double>(
        drake::geometry::Cylinder(radius, height), resolution_hint);

    // Verify the correct size. There are initially 24 tetrahedra that each
    // split into 8 sub tetrahedra.
    const size_t num_tetrahedra = 24 * std::pow(8, level);
    EXPECT_EQ(mesh.num_elements(), num_tetrahedra);

    // Verify that the volume monotonically converges towards the exact volume
    // of the given cylinder.
    const double volume = mesh.CalcVolume();
    const double error = cylinder_volume - volume;

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
GTEST_TEST(MakeCylinderVolumeMesh, EulerCharacteristic) {
  const double height = 2;
  const double radius = 1;

  const int expected_euler_characteristic = 1;

  for (const double resolution_hint : {2., 1., 0.5, 0.25, 0.125, 0.0625}) {
    auto mesh = MakeCylinderVolumeMesh<double>(
        drake::geometry::Cylinder(radius, height), resolution_hint);

    EXPECT_EQ(ComputeEulerCharacteristic(mesh), expected_euler_characteristic);
  }
}

// Smoke test only. Assume correctness of MakeCylinderVolumeMesh() and
// ConvertVolumeToSurfaceMesh(). The resolution_hint 3 times the
// radius of the cylinder produces a triangular prism with 12 triangles and
// 8 vertices, which is the coarsest surface mesh that our algorithm can
// produce. The positions (● in the picture below) of the 8 vertices look
// like this:
//
//                +Z   -X
//                 |   /
//                 |  /
//          ●      | /    ●
//                 |/
//  -Y-------------●-----------+Y
//                /|
//               / |
//              ●  |
//             /   |
//           +X    |  -X
//                 |  /
//          ●      | /    ●
//                 |/
//  -Y-------------●-----------+Y
//                /|
//               / |
//              ●  |
//             /   |
//           +X    |
//                -Z
//
GTEST_TEST(MakeCylinderSurfaceMesh, GenerateSurface) {
  const double radius = 1.0;
  const double length = 2.0;
  const double resolution_hint = 3.0;
  const Cylinder cylinder(radius, length);
  TriangleSurfaceMesh<double> surface_mesh =
      MakeCylinderSurfaceMesh<double>(cylinder, resolution_hint);
  EXPECT_EQ(surface_mesh.num_triangles(), 12);
  EXPECT_EQ(surface_mesh.num_vertices(), 8);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
