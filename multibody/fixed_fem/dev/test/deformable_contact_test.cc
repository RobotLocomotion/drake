#include "drake/multibody/fixed_fem/dev/deformable_contact.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/fixed_fem/dev/deformable_contact_data.h"
#include "drake/multibody/fixed_fem/dev/deformable_rigid_contact_pair.h"
#include "drake/multibody/fixed_fem/dev/mesh_utilities.h"

namespace drake {
namespace multibody {
namespace fem {
namespace {

using Eigen::Vector3d;
using geometry::TriangleSurfaceMesh;
using geometry::VolumeElement;
using geometry::VolumeMesh;
using geometry::VolumeMeshFieldLinear;
using geometry::internal::Bvh;
using geometry::internal::DeformableVolumeMesh;
using geometry::internal::Obb;
using std::vector;

/* The MakePyramidSurface() method is stolen from mesh_intersection_test.cc. */

/* Generates a simple surface mesh of a pyramid with vertices on the
 coordinate axes and the origin like this:

                +Z   -X
                 |   /
              v5 ●  ● v3
                 | /
        v4    v0 |/
  -Y-----●-------●------●---+Y
                /      v2
               /
              ● v1
             /
           +X
*/
template <typename T>
TriangleSurfaceMesh<T> MakePyramidSurface() {
  const int face_data[8][3] = {// The top four faces share the apex vertex v5.
                               {1, 2, 5},
                               {2, 3, 5},
                               {3, 4, 5},
                               {4, 1, 5},
                               // The bottom four faces share the origin v0.
                               {4, 3, 0},
                               {3, 2, 0},
                               {2, 1, 0},
                               {1, 4, 0}};
  vector<geometry::SurfaceTriangle> faces;
  for (auto& face : face_data) {
    faces.emplace_back(face);
  }
  // clang-format off
  vector<Vector3<T>> vertices = {
      { 0,  0, 0},
      { 1,  0, 0},
      { 0,  1, 0},
      {-1,  0, 0},
      { 0, -1, 0},
      { 0,  0, 1}
  };
  // clang-format on
  return TriangleSurfaceMesh<T>(std::move(faces), std::move(vertices));
}

/* Returns true if
 1. A.size() == B.size(), and
 2. there exists a permutation of A, Ap, such that maxₙ|Ap[i](n)-B[i](n)| <
    kEps for all i = 0, ..., A.size()-1.
 Returns false otherwise. */
template <typename T>
bool CompareSetOfVector3s(const std::vector<Vector3<T>>& A,
                          const std::vector<Vector3<T>>& B,
                          const double& kEps) {
  return std::is_permutation(A.begin(), A.end(), B.begin(), B.end(),
                             [kEps](const Vector3<T>& a, const Vector3<T>& b) {
                               return CompareMatrices(a, b, kEps);
                             });
}

/* Returns the contact surface between the rigid pyramid surface (see
 MakePyramidSurface()) and the deforamble octahedron volume (see
 MakeOctahedronVolumeMesh()) where the pose of the rigid mesh in the deformable
 mesh's frame is given by X_DR. */
template <typename T>
DeformableContactSurface<T> MakeDeformableContactSurface(
    const math::RigidTransform<T>& X_DR) {
  const DeformableVolumeMesh<T> volume_D(MakeOctahedronVolumeMesh<T>());
  /* Deformable contact assumes the rigid surface is double-valued, regardless
   of the scalar value for the volume mesh.  */
  const TriangleSurfaceMesh<double> surface_R = MakePyramidSurface<double>();
  const Bvh<Obb, TriangleSurfaceMesh<double>> bvh_R(surface_R);
  return ComputeTetMeshTriMeshContact<T>(volume_D, surface_R, bvh_R, X_DR);
}

/* Limited test to show correctness of ComputeTetMeshTriMeshContact<T>().
 Given a simple, tractable set of input meshes, we analytically compute the
 expected contact data and compare it against the result from
 ComputeTetMeshTriMeshContact<T>(). */
template <typename T>
void TestComputeTetMeshTriMeshContact() {
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  const auto X_DR = math::RigidTransform<T>(Vector3<T>(0, 0, 0.5));
  const DeformableContactSurface<T> contact_D =
      MakeDeformableContactSurface<T>(X_DR);
  /* Move the rigid pyramid up, so only its square base intersects the top
   pyramidal region of the deformable octahedron. The resulting implicit contact
   surface is made up of 4 triangles. The portion of the contact surface inside
   the tetrahedron v0v1v2v3 is given by the triangle c0c1c2. We can calculate
   the positions of the c0, c1 and c2 in the deformable octahedron frame: c0 =
   (0, -0.5, 0.5), c1 = (0, 0, 0.5), c2 = (0.5, 0, 0.5). So the area of the
   polygon (in this case a triangle) is 0.5 * 0.5 * 0.5 = 0.125, the normal is
   (0, 0, -1), and the centroid has position (1/6, -1/6, 0.5) in the deformable
   octahedron's frame. The three other contact polygons have the same area and
   normal, and their centroids can be calculated in a similar fashion.

                 +Z      -X
                  |      /
               v1 ●     /
                  |    /
           c0     | c1/
             ●----●  /
                 /| /
       v2    c2 ● |/
   -Y----●--------●-----------------+Y
                 /| v0
                / |
           v3  ●  |
              /   |
            +X    |
                 -Z                                            */
  const int kNumPolys = 4;
  EXPECT_EQ(contact_D.num_polygons(), kNumPolys);
  EXPECT_FALSE(contact_D.empty());
  vector<ContactPolygonData<T>> contact_data;
  for (int i = 0; i < kNumPolys; ++i) {
    contact_data.push_back(contact_D.polygon_data(i));
  }
  /* Verify the areas and the normals are as expected. */
  const double expected_area = 0.125;
  const Vector3<T> expected_nhat_D(0, 0, -1);
  for (int i = 0; i < kNumPolys; ++i) {
    if constexpr (std::is_same_v<T, double>)
      EXPECT_DOUBLE_EQ(contact_data[i].area, expected_area);
    else
      EXPECT_DOUBLE_EQ(contact_data[i].area.value(), expected_area);
    EXPECT_TRUE(
        CompareMatrices(contact_data[i].unit_normal, expected_nhat_D, kEps));
  }
  /* Verify the centroids in cartesian coordinates are as expected. */
  vector<Vector3<T>> calculated_centroids_D;
  for (int i = 0; i < kNumPolys; ++i) {
    calculated_centroids_D.push_back(contact_data[i].centroid);
  }

  const vector<Vector3<T>> expected_centroids_D{{-1. / 6., -1. / 6., 0.5},
                                                {-1. / 6., 1. / 6., 0.5},
                                                {1. / 6., -1. / 6., 0.5},
                                                {1. / 6., 1. / 6., 0.5}};
  EXPECT_TRUE(
      CompareSetOfVector3s(calculated_centroids_D, expected_centroids_D, kEps));

  /* Verify the centroids in barycentric coordinates are as expected. */
  const VolumeMesh<T> volume_D = MakeOctahedronVolumeMesh<T>();
  calculated_centroids_D.clear();
  for (int i = 0; i < kNumPolys; ++i) {
    const Vector4<T> b_centroid = contact_data[i].b_centroid;
    const int tet_index = contact_data[i].tet_index;
    const VolumeElement& tet = volume_D.element(tet_index);
    Vector3<T> centroid_D(0, 0, 0);
    /* Calculate the centroid in cartesian coordinate by interpolating the
     positions of the tet vertices with the barycentric weights. */
    for (int j = 0; j < 4; ++j) {
      centroid_D += b_centroid(j) * volume_D.vertex(tet.vertex(j));
    }
    calculated_centroids_D.push_back(centroid_D);
  }
  EXPECT_TRUE(
      CompareSetOfVector3s(calculated_centroids_D, expected_centroids_D, kEps));
}

GTEST_TEST(DeformableContactTest, ComputeTetMeshTriMeshContactDouble) {
  TestComputeTetMeshTriMeshContact<double>();
}

GTEST_TEST(DeformableContactTest, ComputeTetMeshTriMeshContactAutoDiff) {
  TestComputeTetMeshTriMeshContact<AutoDiffXd>();
}

/* Verifies that ComputeTetMeshTriMeshContact() gives sensible results when the
 contact polygon is not a triangle. */
GTEST_TEST(DeformableContactTest, NonTriangleContactPolygon) {
  /* Creates a surface mesh with a single, large-enough, triangle in the
   z-plane. */
  int face[3] = {0, 1, 2};
  vector<geometry::SurfaceTriangle> faces;
  faces.emplace_back(face);
  vector<Vector3<double>> tri_vertices = {{10, 0, 0}, {-5, 5, 0}, {-5, -5, 0}};
  const TriangleSurfaceMesh<double> surface_R(std::move(faces),
                                              std::move(tri_vertices));
  const Bvh<Obb, TriangleSurfaceMesh<double>> bvh_R(surface_R);

  /* Creates a tetrahedral mesh with a single tet whose intersection with the
   surface mesh is a axis-aligned unit square [-0.5, 0.5]x[-0.5, 0.5]x{0} in the
   z-plane centered at the origin.  The tetrahedron is also centered at the
   origin. One of its edge is on the z=-1 plane and parallel to the x axis.
   Another edge is on the z=1 plane and parallel to the y axis. The remaining
   four edges connect vertices of these two edges together. */
  const int tet[4] = {0, 1, 2, 3};
  vector<VolumeElement> tets;
  tets.emplace_back(tet);
  const Vector3<double> tet_vertex_data[4] = {
      {1, 0, -1}, {-1, 0, -1}, {0, -1, 1}, {0, 1, 1}};
  vector<Vector3d> tet_vertices;
  for (const auto& vertex : tet_vertex_data) {
    tet_vertices.emplace_back(vertex);
  }
  const DeformableVolumeMesh<double> volume_D(
      VolumeMesh<double>(std::move(tets), std::move(tet_vertices)));

  const DeformableContactSurface<double> contact_D =
      ComputeTetMeshTriMeshContact<double>(volume_D, surface_R, bvh_R,
                                           math::RigidTransformd());
  ASSERT_EQ(contact_D.num_polygons(), 1);
  const ContactPolygonData<double>& data = contact_D.polygon_data(0);
  constexpr double kTol = 4.0 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(data.b_centroid.sum(), 1.0, kTol);
  EXPECT_EQ(data.tet_index, 0);
  EXPECT_TRUE(CompareMatrices(data.unit_normal, Vector3d(0, 0, 1), kTol));
  EXPECT_TRUE(CompareMatrices(data.centroid, Vector3d(0, 0, 0), kTol));
}


const DeformableBodyIndex kDeformableBodyIndex(2);
/* Makes a DeformableContactData with a single contact pair using the given
 `contact_surface`. Unused parameters are set to arbitrary values. */
internal::DeformableContactData<double> MakeDeformableContactData(
    DeformableContactSurface<double> contact_surface,
    internal::ReferenceDeformableGeometry<double> deformable_geometry) {
  const geometry::GeometryId dummy_rigid_id;
  const double dummy_stiffness = 0;
  const double dummy_dissipation = 0;
  const double dummy_friction = 0;
  const internal::DeformableRigidContactPair<double> contact_pair(
      std::move(contact_surface), dummy_rigid_id, kDeformableBodyIndex,
      dummy_stiffness, dummy_dissipation, dummy_friction);
  return internal::DeformableContactData<double>({contact_pair},
                                                 deformable_geometry);
}

GTEST_TEST(DeformableContactTest, DeformableContactData) {
  /* Recall that the octahedron looks like.
                  +Z   -X
                   |   /
                v5 ●  ● v3
                   | /
         v4     v0 |/
    -Y----●--------●------●----+Y
                  /|      v2
                 / |
             v1 ●  ● v6
               /   |
             +X    |
                  -Z
   Rotate the the rigid pyramid around x-axis by -90 degrees (right-handed) and
   then shift it along the positive y-axis so that only its square base
   intersects the right pyramidal region of the deformable octahedron. As a
   result, all vertices except v4 are participating in contact. */
  const auto X_DR = math::RigidTransformd(
      math::RotationMatrix<double>::MakeXRotation(-M_PI / 2),
      Vector3<double>(0, 0.5, 0));
  DeformableContactSurface<double> contact_surface =
      MakeDeformableContactSurface<double>(X_DR);
  const internal::DeformableContactData<double> contact_data =
      MakeDeformableContactData(std::move(contact_surface),
                                MakeOctahedronDeformableGeometry<double>());

  EXPECT_EQ(contact_data.num_contact_pairs(), 1);
  /* v0, v1, v2, v3, v5, v6 are participating in contact so they get new indexes
   0, 1, 2, 3, 4, 5. v4 is not participating in contact and gets new index 6. */
  EXPECT_EQ(contact_data.num_vertices_in_contact(), 6);
  const std::vector<int> permuted_vertex_indexes = {0, 1, 2, 3, 6, 4, 5};
  EXPECT_EQ(contact_data.permuted_vertex_indexes(), permuted_vertex_indexes);
  /* Verify that permuted_vertex_indexes() and permuted_to_original_indexes()
   are inverses to each other. */
  const std::vector<int>& permuted_to_original_indexes =
      contact_data.permuted_to_original_indexes();
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(i, permuted_vertex_indexes[permuted_to_original_indexes[i]]);
    EXPECT_EQ(i, permuted_to_original_indexes[permuted_vertex_indexes[i]]);
  }
  EXPECT_EQ(contact_data.deformable_body_index(), kDeformableBodyIndex);
}

GTEST_TEST(DeformableContactTest, EmptyDeformableContactData) {
  /* Move the rigid pyramid way down, so there is no contact. */
  const auto X_DR = math::RigidTransformd(Vector3<double>(0, 0, -15));
  DeformableContactSurface<double> contact_surface =
      MakeDeformableContactSurface<double>(X_DR);
  const internal::DeformableContactData<double> contact_data =
      MakeDeformableContactData(std::move(contact_surface),
                                MakeOctahedronDeformableGeometry<double>());

  EXPECT_EQ(contact_data.num_contact_pairs(), 1);
  EXPECT_EQ(contact_data.num_vertices_in_contact(), 0);
  const std::vector<int> vertex_indexes = {0, 1, 2, 3, 4, 5, 6};
  EXPECT_EQ(contact_data.permuted_vertex_indexes(), vertex_indexes);
  EXPECT_EQ(contact_data.permuted_to_original_indexes(), vertex_indexes);
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
