#include "drake/geometry/proximity/make_cylinder_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

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
// @pre No vertex of the tetrahedron is outside the box.
//
// Here we omit the frame notations because everything is expressed in the
// frame of the box. It might change in the future.
bool IsTetrahedronRespectingMa(const VolumeElement& tetrahedron,
                               const VolumeMesh<double>& mesh,
                               const Cylinder& cylinder,
                               const double tolerance) {
  auto distance_to_boundary =
      [&cylinder](const Vector3d& point_in_cylinder) -> double {
    const double x = point_in_cylinder.x();
    const double y = point_in_cylinder.y();
    const double z = point_in_cylinder.z();
    const double radial_distance = cylinder.radius() - sqrt(x * x + y * y);
    const double top_z = cylinder.length() / 2;
    const double bottom_z = -top_z;
    const double bottom_distance = z - bottom_z;
    const double top_distance = top_z - z;
    return std::min({radial_distance, bottom_distance, top_distance});
  };

  // Face 0 is the bottom circular cap of the cylinder.
  // Face 1 is the top circular cap of the cylinder.
  // Face 2 is the side cylindrical surface.
  const int num_cylinder_faces = 3;
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

  // TODO(DamrongGuoy): When PR #14279 (Migrates IsTetrahedronRespectingMa() to
  //  proximity_utilities package) lands, use the shared code instead of the
  //  following code.

  // Each vertex may have multiple closest faces when the vertex is on the
  // medial axis.
  std::vector<int> closest_faces[mesh.kVertexPerElement];
  for (int v = 0; v < mesh.kVertexPerElement; ++v) {
    const Vector3d r_MV = mesh.vertex(tetrahedron.vertex(v)).r_MV();
    const double dist = distance_to_boundary(r_MV);
    for (int f = 0; f < num_cylinder_faces; ++f) {
      if (distance_to_boundary_face(f, r_MV) - dist <= tolerance) {
        closest_faces[v].push_back(f);
      }
    }
  }
  // Check that at least one face is closest
  for (int f = 0; f < num_cylinder_faces; ++f) {
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
// @retval true when all conditions pass.
//
// Here we omit the frame notations because everything is expressed in the
// frame of the cylinder.
bool VerifyCylinderMeshWithMa(const VolumeMesh<double>& mesh,
                              const Cylinder& cylinder,
                              const int num_vertex_per_circle) {
  // A. The mesh is conforming.
  // A1. The mesh has unique vertices.
  const int num_vertices = mesh.num_vertices();
  for (VolumeVertexIndex i(0); i < num_vertices; ++i) {
    for (VolumeVertexIndex j(i + 1); j < num_vertices; ++j) {
      const bool vertex_is_unique =
          mesh.vertex(i).r_MV() != mesh.vertex(j).r_MV();
      EXPECT_TRUE(vertex_is_unique) << "The mesh has duplicated vertices.";
      if (!vertex_is_unique) {
        return false;
      }
    }
  }
  // A2. Euler characteristic = 1. This is a necessary condition (but may not
  //     be sufficient) for conforming tetrahedra (two tetrahedra intersect in
  //     their shared face, or shared edge, or shared vertex, or not at all.
  //     There is no partial overlapping of two tetrahedra.).
  const int euler_characteristic = ComputeEulerCharacteristic(mesh);
  EXPECT_EQ(1, euler_characteristic)
      << "The mesh's tetrahedra are not conforming because the mesh's Euler "
         "characteristic is "
      << euler_characteristic << " instead of 1.";
  if (euler_characteristic != 1) {
    return false;
  }

  // B. The mesh conforms to the cylinder.
  // B1. Every mesh vertex is in the cylinder.
  const double half_length = cylinder.length() / 2.;
  const double squared_radius = cylinder.radius() * cylinder.radius();
  const double tolerance = DistanceToPointRelativeTolerance(
      std::max(half_length, cylinder.radius()));
  for (const VolumeVertex<double>& v : mesh.vertices()) {
    const double x = v.r_MV().x();
    const double y = v.r_MV().y();
    const double z = v.r_MV().z();
    bool is_inside_or_on_boundary = abs(z) < half_length + tolerance &&
                                    x * x + y * y < squared_radius + tolerance;
    EXPECT_TRUE(is_inside_or_on_boundary)
        << "A mesh vertex is outside the cylinder.";
    if (!is_inside_or_on_boundary) {
      return false;
    }
  }
  // B2. The volume of the mesh approximates the volume of the cylinder.
  const double mesh_volume = mesh.CalcVolume();
  const double expect_volume = half_length * squared_radius *
                               num_vertex_per_circle *
                               sin(2. * M_PI / num_vertex_per_circle);
  const double volume_tolerance = 10. * tolerance;
  EXPECT_NEAR(mesh_volume, expect_volume, volume_tolerance)
      << "The mesh's volume does not approximate the cylinder's volume enough.";
  if (abs(mesh_volume - expect_volume) > volume_tolerance) {
    return false;
  }

  // C. The mesh conforms to the cylinder's medial axis.
  // C1. No tetrahedron has all four vertices on the cylinder's boundary, i.e.,
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
    EXPECT_TRUE(tetrahedron_has_an_interior_vertex)
              << "A tetrahedron has all its vertices on the boundary.";
    if (!tetrahedron_has_an_interior_vertex) {
      return false;
    }
  }
  // C2. No tetrahedron has all four vertices at the same distance to the
  //     cylinder's surface.
  auto distance_to_boundary =
      [&cylinder](const Vector3d& point_in_cylinder) -> double {
        const double x = point_in_cylinder.x();
        const double y = point_in_cylinder.y();
        const double z = point_in_cylinder.z();
        const double radial_distance = cylinder.radius() - sqrt(x * x + y * y);
        const double top_z = cylinder.length() / 2;
        const double bottom_z = -top_z;
        const double bottom_distance = z - bottom_z;
        const double top_distance = top_z - z;
        return std::min({radial_distance, bottom_distance, top_distance});
      };
  for (const VolumeElement& tetrahedron : mesh.tetrahedra()) {
    const double distance_v0 =
        distance_to_boundary(mesh.vertex(tetrahedron.vertex(0)).r_MV());
    bool different_distance_from_v0 = false;
    for (int i = 1; i < mesh.kVertexPerElement && !different_distance_from_v0;
         ++i) {
      different_distance_from_v0 =
          tolerance <
          abs(distance_v0 -
              distance_to_boundary(mesh.vertex(tetrahedron.vertex(i)).r_MV()));
    }
    EXPECT_TRUE(different_distance_from_v0)
        << "A tetrahedron has all vertices at the same distances to"
           " the cylinder's surface.";
    if (!different_distance_from_v0) {
      return false;
    }
  }
  // C3. Each tetrahedron conforms to MA. Assume every mesh vertex is in
  //     the cylinder.
  for (const VolumeElement& tetrahedron : mesh.tetrahedra()) {
    bool tetrahedron_conform_to_MA =
        IsTetrahedronRespectingMa(tetrahedron, mesh, cylinder, tolerance);
    EXPECT_TRUE(tetrahedron_conform_to_MA)
        << "A tetrahedron does not conform to the medial axis of the "
           "cylinder.";
    if (!tetrahedron_conform_to_MA) {
      return false;
    }
  }

  return true;
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
  EXPECT_TRUE(VerifyCylinderMeshWithMa(mesh, cylinder, num_vertex_per_circle));
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
  EXPECT_TRUE(VerifyCylinderMeshWithMa(mesh, cylinder, num_vertex_per_circle));
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
  EXPECT_TRUE(VerifyCylinderMeshWithMa(mesh, cylinder, num_vertex_per_circle));
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

// The test size is increased to "medium" so that debug builds are successful
// in CI. See BUILD.bazel.
GTEST_TEST(MakeCylinderVolumeMesh, FinestMesh) {
  const double radius = 1.0;
  const double length = 2.0;
  // The initial mesh of a cylinder with radius 1 and length 2 has 24
  // tetrahedra. Each refinement level increases the number of tetrahedra by
  // a factor of 8. A refinement level ℒ that could increase tetrahedra beyond
  // 100 million is ℒ = 8, which will give 24 * 8⁸ = 402,653,184 tetrahedra.
  // (However, we will confirm later that the algorithm will limit the number
  // of tetrahedra within 100 million, so it should give
  // 24 * 8⁷ = 50,331,648 tetrahedra instead.) We will calculate the
  // resolution hint that could trigger ℒ = 8.
  //     Each edge of the mesh on the boundary circle of the top and bottom
  // caps is a chord of the circle with a central angle θ. The edge length e
  // is calculated from θ and radius r as:
  //     e = 2⋅r⋅sin(θ/2)
  // The central angle is initially π/2, and each level of refinement
  // decreases it by a factor of 2. For ℒ = 8, we have:
  //     θ = (π/2)/(2⁸) = π/512
  // Thus, we have e = 2 sin (π/512) = 0.0122717...
  //     We use a resolution hint smaller than e in the sixth decimal digit to
  // request 402,653,184 tetrahedra, but the algorithm should limit the
  // number of tetrahedra within 100 million. As a result, we should have
  // 50,331,648 tetrahedra instead.
  const double resolution_hint_limit = 1.2271e-2;
  auto mesh_limit = MakeCylinderVolumeMesh<double>(Cylinder(radius, length),
                                                   resolution_hint_limit);
  EXPECT_EQ(50331648, mesh_limit.num_elements());

  // Confirm that going ten times finer resolution hint does not increase the
  // number of tetrahedra.
  const double resolution_hint_finer = 1.2271e-3;
  auto mesh_same = MakeCylinderVolumeMesh<double>(Cylinder(radius, length),
                                                   resolution_hint_finer);
  EXPECT_EQ(50331648, mesh_same.num_elements());
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
// ConvertVolumeToSurfaceMesh(). The resolution_hint larger than √2 times the
// radius of the cylinder produces a rectangular prism with 24 triangles and
// 14 vertices, which is the coarsest surface mesh that our algorithm can
// produce. The positions (● in the picture below) of the 14 vertices look
// like this:
//
//                +Z   -X
//                 |   /
//                 |  ●
//                 | /
//                 |/
//  -Y-----●-------●-------●---+Y
//                /|
//               / |
//              ●  |
//             /   |
//           +X    |   -X
//                 |   /
//                 |  ●
//                 | /
//                 |/
//  -Y-----●-------+-------●---+Y
//                /|
//               / |
//              ●  |
//             /   |
//           +X    |    -X
//                 |   /
//                 |  ●
//                 | /
//                 |/
//  -Y-----●-------●-------●---+Y
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
  const double resolution_hint = 1.5;
  const Cylinder cylinder(radius, length);
  SurfaceMesh<double> surface_mesh =
      MakeCylinderSurfaceMesh<double>(cylinder, resolution_hint);
  EXPECT_EQ(surface_mesh.num_faces(), 24);
  EXPECT_EQ(surface_mesh.num_vertices(), 14);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
