#include "drake/geometry/mesh_deformation_interpolator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;

VolumeMesh<double> MakeSingleTetVolumeMesh() {
  std::vector<VolumeElement> elements{VolumeElement{0, 1, 2, 3}};
  std::vector<Vector3d> vertices{Vector3d{0, 0, 0}, Vector3d::UnitX(),
                                 Vector3d::UnitY(), Vector3d::UnitZ()};
  return VolumeMesh(std::move(elements), std::move(vertices));
}

/* Makes an octahedron volume mesh.
 The octahedron looks like this in its geometry frame, F.
                  +Fz   -Fx
                   |   /
                   v5 v3
                   | /
                   |/
   -Fy---v4------v0+------v2---+ Fy
                  /| Fo
                 / |
               v1  v6
               /   |
             +Fx   |
                  -Fz
*/
VolumeMesh<double> MakeOctahedronVolumeMesh() {
  return geometry::internal::MakeSphereMeshLevel0<double>().first;
}

GTEST_TEST(BarycentricInterpolatorTest, ConstructAndInterpolate) {
  std::vector<Vector3d> positions{Vector3d(
      0.25, 0.25, 0.25)};  // barycentric coordinate (0.25, 0.25, 0.25, 0.25).
  const BarycentricInterpolator interpolator(positions,
                                             MakeSingleTetVolumeMesh());
  // Arbirtrary q for 4 vertices of the control mesh.
  const VectorXd q = VectorXd::LinSpaced(12, 0.0, 1.0);
  const VectorXd interpolated_q = interpolator(q);
  EXPECT_EQ(interpolated_q.size(), 3);
  Vector3d expected_q = Vector3d::Zero();
  for (int i = 0; i < 4; ++i) {
    expected_q += 0.25 * q.segment<3>(3 * i);
  }
  EXPECT_TRUE(CompareMatrices(interpolated_q, expected_q));
  // Throws if the size of q for the control mesh is the wrong size.
  EXPECT_THROW(interpolator(VectorXd::LinSpaced(9, 0.0, 1.0)), std::exception);
}

GTEST_TEST(BarycentricInterpolatorTest, PassivePointOutOfBound) {
  std::vector<Vector3d> positions(1);
  // Point outside of the tet.
  positions[0] = Vector3d(-1e-4, 0, 0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      BarycentricInterpolator(positions, MakeSingleTetVolumeMesh()),
      ".*driven point lies outside.*");
}

GTEST_TEST(VertexSampler, ConstructAndInterpolate) {
  const VolumeMesh<double> control_mesh = MakeOctahedronVolumeMesh();
  const VertexSampler selector(std::vector<int>{3}, control_mesh);
  // Arbitrary q for 7 vertices of the control mesh.
  const VectorXd q = VectorXd::LinSpaced(21, 0.0, 1.0);
  const VectorXd interpolated_q = selector(q);
  EXPECT_TRUE(CompareMatrices(interpolated_q, q.segment<3>(3 * 3)));
  // Throws if the size of q for the control mesh is the wrong size.
  EXPECT_THROW(selector(VectorXd::LinSpaced(9, 0.0, 1.0)), std::exception);
}

GTEST_TEST(VertexSampler, ConstructorFailures) {
  const VolumeMesh<double> control_mesh = MakeOctahedronVolumeMesh();
  // No vertex selected.
  EXPECT_THROW(VertexSampler(std::vector<int>{}, control_mesh), std::exception);
  // Duplicated vertices.
  EXPECT_THROW(VertexSampler(std::vector<int>{1, 1}, control_mesh),
               std::exception);
  // Vertices out of bound.
  EXPECT_THROW(VertexSampler(std::vector<int>{-1}, control_mesh),
               std::exception);
  EXPECT_THROW(VertexSampler(std::vector<int>{7}, control_mesh),
               std::exception);
  // Vertices not sorted.
  EXPECT_THROW(VertexSampler(std::vector<int>{3, 2, 1}, control_mesh),
               std::exception);
}

/* Tests construction of DrivenTriangleMesh and the vertex
 position/normal computation for the driven vertices. */
GTEST_TEST(DrivenTriangleMesh, VertexPositionAndNormal) {
  const VolumeMesh<double> control_mesh = MakeOctahedronVolumeMesh();
  /* We create the surface mesh so that it's the surface of the tetrahedron
   formed by v0, v1, v2, and v5. */
  std::vector<Vector3d> vertices;
  vertices.emplace_back(0, 0, 0);  // v0
  vertices.emplace_back(1, 0, 0);  // v1
  vertices.emplace_back(0, 1, 0);  // v2
  vertices.emplace_back(0, 0, 1);  // v5
  std::vector<SurfaceTriangle> elements;
  elements.emplace_back(0, 1, 3);  // v0v1v5
  elements.emplace_back(0, 2, 1);  // v0v2v1
  elements.emplace_back(0, 3, 2);  // v0v5v2
  elements.emplace_back(1, 2, 3);  // v1v2v5
  TriangleSurfaceMesh<double> surface_mesh(std::move(elements),
                                           std::move(vertices));
  DrivenTriangleMesh driven_mesh(surface_mesh, control_mesh);
  // Move the vertex of the control mesh's v5 in the +z-direction by 1.
  VectorXd q(21);
  // clang-format off
  q << 0, 0, 0,  // v0 doesn't move
       1, 0, 0,  // v1 doesn't move
       0, 1, 0,  // v2 doesn't move
       8, 4, 7,  // v3 moved to arbitrary location, doesn't affect driven mesh.
       4, 8, 7,  // v4 moved to arbitrary location, doesn't affect driven mesh.
       0, 0, 2,  // v5 moved to prescribed location
       7, 4, 9;  // v6 moved to arbitrary location, doesn't affect driven mesh.
  // clang-format on

  // GetDrivenVertexPositions() simply concatenates vertex positions. Let's see
  // if it does it.
  const std::vector<Vector3d>& verts = driven_mesh.mesh().vertices();
  Eigen::Map<const VectorXd> expected_q(
      reinterpret_cast<const double*>(verts.data()), 3 * verts.size());
  driven_mesh.SetControlMeshPositions(q);
  EXPECT_TRUE(
      CompareMatrices(expected_q, driven_mesh.GetDrivenVertexPositions()));

  // We record the areas and normals of each of the surface mesh to compute the
  // area-weighted normal.
  std::vector<Vector3d> normals;
  normals.emplace_back(0, -1, 0);  // v0v1v5
  normals.emplace_back(0, 0, -1);  // v0v2v1
  normals.emplace_back(-1, 0, 0);  // v0v5v2
  normals.emplace_back(2, 2, 1);   // v1v2v5
  normals.back().normalize();
  std::vector<double> areas;
  areas.emplace_back(1.0);  // v0v1v5
  areas.emplace_back(0.5);  // v0v2v1
  areas.emplace_back(1.0);  // v0v5v2
  areas.emplace_back(1.5);  // v1v2v5

  std::vector<Vector3<int>> incident_triangles;
  incident_triangles.emplace_back(0, 1, 2);  // v0
  incident_triangles.emplace_back(0, 1, 3);  // v1
  incident_triangles.emplace_back(1, 2, 3);  // v2
  incident_triangles.emplace_back(0, 2, 3);  // v5

  VectorXd expected_normals = VectorXd::Zero(12);
  for (int v = 0; v < 4; ++v) {
    for (int t = 0; t < 3; ++t) {
      const int tri = incident_triangles[v][t];
      expected_normals.segment<3>(3 * v) += areas[tri] * normals[tri];
    }
    expected_normals.segment<3>(3 * v).normalize();
  }
  EXPECT_TRUE(
      CompareMatrices(expected_normals, driven_mesh.GetDrivenVertexNormals()));

  /* Now we test the constructor that explicitly specifies the interpolator. */
  VertexSampler interpolator(std::vector<int>{0, 1, 2, 5}, control_mesh);
  DrivenTriangleMesh another_driven_mesh(
      std::variant<BarycentricInterpolator, VertexSampler>(
          std::move(interpolator)),
      TriangleSurfaceMesh<double>(std::move(surface_mesh)));
  another_driven_mesh.SetControlMeshPositions(q);
  EXPECT_TRUE(CompareMatrices(expected_q,
                              another_driven_mesh.GetDrivenVertexPositions()));
  EXPECT_TRUE(CompareMatrices(expected_normals,
                              another_driven_mesh.GetDrivenVertexNormals()));
}

// We test geometry::internal::MakeDrivenSurfaceMesh on the unit octahedron
// volume mesh to verify that the mesh is driven as expected. The components of
// the function have been individually unit tested; here we just test things are
// correctly wired up.
GTEST_TEST(MakeDrivenSurfaceMesh, OctahedronMesh) {
  const VolumeMesh<double> control_mesh = MakeOctahedronVolumeMesh();
  DrivenTriangleMesh driven_mesh = MakeDrivenSurfaceMesh(control_mesh);
  // The mapping of vertex indices from the surface mesh to the volume mesh
  // should be (0,1,2,3,4,5) -> (1,2,3,4,5,6).
  const std::vector<int> expected_mapping{1, 2, 3, 4, 5, 6};
  const VectorXd control_q =
      VectorXd::LinSpaced(control_mesh.num_vertices() * 3, 0.0, 1.0);
  driven_mesh.SetControlMeshPositions(control_q);
  const VectorXd driven_q = driven_mesh.GetDrivenVertexPositions();
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(control_q.segment<3>(3 * expected_mapping[i]),
              driven_q.segment<3>(3 * i));
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
