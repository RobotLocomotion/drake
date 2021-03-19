#include "drake/multibody/fixed_fem/dev/deformable_contact.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace {

using geometry::SurfaceMesh;
using geometry::VolumeElement;
using geometry::VolumeElementIndex;
using geometry::VolumeMesh;
using geometry::VolumeVertex;
using std::vector;

/* The OctahedronVolume() and MakePyramidSurface() methods are stolen from
 mesh_intersection_test.cc. */

/* Generates a volume mesh of an octahedron comprising of eight tetrahedral
 elements with vertices on the coordinate axes and the origin like this:

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
*/
template <typename T>
VolumeMesh<T> OctahedronVolume() {
  const int element_data[8][4] = {
      // The top four tetrahedrons share the top vertex v5.
      {0, 1, 2, 5},
      {0, 2, 3, 5},
      {0, 3, 4, 5},
      {0, 4, 1, 5},
      // The bottom four tetrahedrons share the bottom vertex v6.
      {0, 2, 1, 6},
      {0, 3, 2, 6},
      {0, 4, 3, 6},
      {0, 1, 4, 6}};
  vector<VolumeElement> elements;
  for (const auto& element : element_data) {
    elements.emplace_back(element);
  }
  // clang-format off
  const Vector3<T> vertex_data[7] = {
      { 0,  0,  0},
      { 1,  0,  0},
      { 0,  1,  0},
      {-1,  0,  0},
      { 0, -1,  0},
      { 0,  0,  1},
      { 0,  0, -1}};
  // clang-format on
  vector<VolumeVertex<T>> vertices;
  for (const auto& vertex : vertex_data) {
    vertices.emplace_back(vertex);
  }
  return VolumeMesh<T>(std::move(elements), std::move(vertices));
}

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
SurfaceMesh<T> MakePyramidSurface() {
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
  vector<geometry::SurfaceFace> faces;
  for (auto& face : face_data) {
    faces.emplace_back(face);
  }
  // clang-format off
  const Vector3<T> vertex_data[6] = {
      { 0,  0, 0},
      { 1,  0, 0},
      { 0,  1, 0},
      {-1,  0, 0},
      { 0, -1, 0},
      { 0,  0, 1}
  };
  // clang-format on
  vector<geometry::SurfaceVertex<T>> vertices;
  for (auto& vertex : vertex_data) {
    vertices.emplace_back(vertex);
  }
  return SurfaceMesh<T>(std::move(faces), std::move(vertices));
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

/* Limited test to show correctness of ComputeTetMeshTriMeshContact<T>().
 Given a simple, tractable set of input meshes, we analytically compute the
 expected contact data and compare it against the result from
 ComputeTetMeshTriMeshContact<T>(). */
template <typename T>
void TestComputeTetMeshTriMeshContact() {
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  const VolumeMesh<T> volume_D = OctahedronVolume<T>();
  /* Deformable contact assumes the rigid surface is double-valued, regardless
   of the scalar value for the volume mesh.  */
  const SurfaceMesh<double> surface_R = MakePyramidSurface<double>();
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
  const auto X_DR = math::RigidTransform<T>(Vector3<T>(0, 0, 0.5));

  const DeformableContactSurface<T> contact_D =
      ComputeTetMeshTriMeshContact<T>(volume_D, surface_R, X_DR);
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
  calculated_centroids_D.clear();
  for (int i = 0; i < kNumPolys; ++i) {
    const Vector4<T> b_centroid = contact_data[i].b_centroid;
    const VolumeElementIndex tet_index = contact_data[i].tet_index;
    const VolumeElement& tet = volume_D.element(tet_index);
    Vector3<T> centroid_D(0, 0, 0);
    /* Calculate the centroid in cartesian coordinate by interpolating the
     positions of the tet vertices with the barycentric weights. */
    for (int j = 0; j < 4; ++j) {
      centroid_D += b_centroid(j) * volume_D.vertex(tet.vertex(j)).r_MV();
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
}  // namespace
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
