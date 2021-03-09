#include "drake/multibody/fixed_fem/dev/deformable_contact.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/mesh_intersection.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace {

using geometry::SurfaceFaceIndex;
using geometry::SurfaceMesh;
using geometry::VolumeElementIndex;
using geometry::VolumeMesh;
using geometry::internal::SurfaceVolumeIntersector;
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
  vector<geometry::VolumeElement> elements;
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
  vector<geometry::VolumeVertex<T>> vertices;
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

/* The per-triangle data we're testing for.  */
template <typename T>
struct TriangleData {
  T area{};
  Vector3<T> normal;
  /* The centroid computed from *triangle* vertex positions.  */
  Vector3<T> centroid;
  /* The centroid computed from weighted combination of tetrahedron vertices. */
  Vector3<T> centroid_from_bary;
};

/* Given a triangle from the DeformableContactSurface's triangle mesh, computes
 some quantities to test the *consistency* between the actual mesh and the
 stored per-triangle quantities reported in the DeformableContactSurface.  */
template <typename T>
TriangleData<T> CalcTriangleData(const SurfaceMesh<T>& surface_D,
                                 SurfaceFaceIndex tri_index,
                                 const VolumeMesh<T>& volume_D,
                                 VolumeElementIndex tet_index,
                                 const Vector4<T>& b_Q) {
  TriangleData<T> data_D;
  const geometry::SurfaceFace& tri = surface_D.element(tri_index);
  const auto& p_DA = surface_D.vertex(tri.vertex(0)).r_MV();
  const auto& p_DB = surface_D.vertex(tri.vertex(1)).r_MV();
  const auto& p_DC = surface_D.vertex(tri.vertex(2)).r_MV();
  data_D.centroid = (p_DA + p_DB + p_DC) / 3;

  const Vector3<T> cross_D = (p_DB - p_DA).cross(p_DC - p_DA);
  const T mag = cross_D.norm();
  data_D.area = mag / 2;
  data_D.normal = cross_D / mag;

  Vector3<T> centroid = Vector3<T>::Zero();
  const auto& tet = volume_D.element(tet_index);
  for (int v = 0; v < 4; ++v) {
    centroid += b_Q(v) * volume_D.vertex(tet.vertex(v)).r_MV();
  }
  data_D.centroid_from_bary = centroid;
  return data_D;
}

/* Limited test to show correctness of ComputeTetMeshTriMeshContact<T>().
 Given a simple, tractable set of input meshes, we compute the surface and
 evaluate it in two ways:

   1. Make sure it's the "right" manifold.
   2. The per-triangle data is consistent with the mesh.
 */
template <typename T>
void TestComputeTetMeshTriMeshContact() {
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  const VolumeMesh<T> volume_D = OctahedronVolume<T>();
  /* FEM contact assumes the rigid surface is double-valued, regardless of the
   scalar value for the volume mesh.  */
  const SurfaceMesh<double> surface_R = MakePyramidSurface<double>();
  /* Move the rigid pyramid up, so only its square base intersects the top
   pyramidal region of the deformable octahedron.  */
  const auto X_DR = math::RigidTransform<T>(Vector3<T>(0, 0, 0.5));

  const DeformableContactSurface<T> contact_D =
      ComputeTetMeshTriMeshContact<T>(volume_D, surface_R, X_DR);
  const SurfaceMesh<T>& mesh_D = contact_D.mesh();

  /* First, we need to test that the mesh in the contact surface is the
   expected manifold. This is *challenging*. Instead, we're going to compute
   the corresponding hydroleastic contact surface and confirm that its mesh
   has the same total area and same geometric centroid. That will be considered
   evidence that the mesh is correct.

   A more direct comparison is impractical because the two meshes, by design,
   tesselate the contact surface differently. The logic to reconcile those two
   different decompositions is more work than it's worth.  */

  /* Construct arguments for hydroelastic query.  */

  /* Construct the output parameters. We only care about the test_mesh, the
   field and gradient will otherwise be ignored.  */
  std::unique_ptr<SurfaceMesh<T>> test_mesh_D{};
  std::unique_ptr<geometry::SurfaceMeshFieldLinear<T, T>> ignored_field;
  vector<Vector3<T>> ignored_gradient;

  /* Construct the input parameters.  */
  geometry::internal::Bvh<VolumeMesh<T>> bvh_volume_D(volume_D);
  vector<T> values(volume_D.num_vertices(), T(0));
  geometry::VolumeMeshFieldLinear<T, T> field_D(
      "ignored", move(values), &volume_D, true /* calculate_gradient */);
  /* Hydroelastic intersection assumes the surface has the same scalar value as
   the volume mesh; reconstruct a T-valued triangle mesh.  */
  const SurfaceMesh<T> surface_R_T = MakePyramidSurface<T>();
  geometry::internal::Bvh<SurfaceMesh<T>> bvh_surface(surface_R_T);

  geometry::internal::SurfaceVolumeIntersector<T>().SampleVolumeFieldOnSurface(
      field_D, bvh_volume_D, surface_R_T, bvh_surface, X_DR, &test_mesh_D,
      &ignored_field, &ignored_gradient);

  EXPECT_NEAR(ExtractDoubleOrThrow(test_mesh_D->total_area()),
              ExtractDoubleOrThrow(mesh_D.total_area()), kEps);
  EXPECT_TRUE(
      CompareMatrices(test_mesh_D->centroid(), mesh_D.centroid(), kEps));

  /* Second, we confirm the data is internally consistent. I.e.,
      - normal is perpendicular to triangle.
      - area is as expected.
      - centroid in Cartesian is correct; truly the average position.
      - centroid in Barycentric produces Cartesian point (with tet
        index).  */
  for (SurfaceFaceIndex f(0); f < mesh_D.num_faces(); ++f) {
    const ContactTriangleData<T>& f_data_D = contact_D.triangle_data(f);
    const TriangleData<T> expected_D = CalcTriangleData(
        mesh_D, f, volume_D, VolumeElementIndex(f_data_D.tet_index),
        f_data_D.b_centroid);

    EXPECT_NEAR(ExtractDoubleOrThrow(mesh_D.area(f)),
                ExtractDoubleOrThrow(expected_D.area), 1e-15);
    EXPECT_TRUE(
        CompareMatrices(mesh_D.face_normal(f), expected_D.normal, kEps));

    /* Make sure centroid *is* the centroid of the triangle.  */
    EXPECT_TRUE(CompareMatrices(f_data_D.centroid, expected_D.centroid, kEps));
    /* Confirm that b_centroid + volume mesh vertices = centroid.  */
    EXPECT_TRUE(CompareMatrices(f_data_D.centroid,
                                expected_D.centroid_from_bary, kEps));
  }
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
