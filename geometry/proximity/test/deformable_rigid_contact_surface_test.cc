#include "drake/geometry/proximity/deformable_rigid_contact_surface.h"

#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace internal {

namespace {

const GeometryId kRigidId = GeometryId::get_new_id();
const GeometryId kDeformableId = GeometryId::get_new_id();
const double kSignedDistance = -0.1;
const int kTetIndex = 42;

/* Helper function to facilitate MakeDeformableRigidContactSurface(). */
template <typename T>
std::unique_ptr<PolygonSurfaceMesh<T>> MakePolygonSurfaceMesh() {
  // Make a polygon mesh with a single triangle with vertices at (1, 0, 0),
  // (0, 1, 0), and (0, 0, 1).
  return std::make_unique<PolygonSurfaceMesh<T>>(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<T>>{
          Vector3<T>::UnitX(),
          Vector3<T>::UnitY(),
          Vector3<T>::UnitZ(),
      });
}

/* Builds a DeformableRigidContactSurface for testing purpose.
 The contact surface that involves a single contact polygon and a single contact
 point with a signed distance of -0.1. The single contact polygon is contained
 in the tetrahedron with index kTetIndex. See constructor of
 DeformableRigidContactSurface. */
template <typename T>
DeformableRigidContactSurface<T> MakeDeformableRigidContactSurface() {
  std::unique_ptr<PolygonSurfaceMesh<T>> polygon_surface_mesh =
      MakePolygonSurfaceMesh<T>();
  PolygonSurfaceMeshFieldLinear<T, T> sdf(
      // Constant field with dummy value -0.1.
      std::vector<T>{-0.1, -0.1, -0.1}, polygon_surface_mesh.get(),
      // Constant field values means the gradient is zero.
      std::vector<Vector3<T>>{Vector3<T>::Zero()});

  return DeformableRigidContactSurface<T>(
      std::move(polygon_surface_mesh), sdf, std::vector<int>{kTetIndex},
      std::vector<Vector4<T>>{Vector4<T>{0.1, 0.2, 0.3, 0.4}}, kRigidId,
      kDeformableId);
}

using ScalarTypes = ::testing::Types<double>;
TYPED_TEST_SUITE(DeformableRigidContactSurfaceTest, ScalarTypes);
template <typename T>
class DeformableRigidContactSurfaceTest : public ::testing::Test {};

TYPED_TEST(DeformableRigidContactSurfaceTest, GeometryIds) {
  using T = TypeParam;
  DeformableRigidContactSurface<T> dut = MakeDeformableRigidContactSurface<T>();
  EXPECT_EQ(dut.deformable_id(), kDeformableId);
  EXPECT_EQ(dut.rigid_id(), kRigidId);
}

TYPED_TEST(DeformableRigidContactSurfaceTest, TetrahedronIndices) {
  using T = TypeParam;
  DeformableRigidContactSurface<T> dut = MakeDeformableRigidContactSurface<T>();
  ASSERT_EQ(dut.tetrahedron_indices().size(), 1);
  EXPECT_EQ(dut.tetrahedron_indices()[0], kTetIndex);
}

TYPED_TEST(DeformableRigidContactSurfaceTest, BarycentricCentroids) {
  using T = TypeParam;
  DeformableRigidContactSurface<T> dut = MakeDeformableRigidContactSurface<T>();
  ASSERT_EQ(dut.barycentric_centroids().size(), 1);
  EXPECT_EQ(dut.barycentric_centroids()[0], Vector4<T>(0.1, 0.2, 0.3, 0.4));
}

TYPED_TEST(DeformableRigidContactSurfaceTest, NumberOfContactPoints) {
  using T = TypeParam;
  DeformableRigidContactSurface<T> dut = MakeDeformableRigidContactSurface<T>();
  EXPECT_EQ(dut.num_contact_points(), 1);
}

TYPED_TEST(DeformableRigidContactSurfaceTest, EvaluatePenetrationDistance) {
  using T = TypeParam;
  DeformableRigidContactSurface<T> dut = MakeDeformableRigidContactSurface<T>();
  EXPECT_EQ(dut.penetration_distance(0), kSignedDistance);
}

TYPED_TEST(DeformableRigidContactSurfaceTest, ContactSurfaceMesh) {
  using T = TypeParam;
  DeformableRigidContactSurface<T> dut = MakeDeformableRigidContactSurface<T>();
  std::unique_ptr<PolygonSurfaceMesh<T>> contact_surface_mesh_W =
      MakePolygonSurfaceMesh<T>();
  EXPECT_TRUE(
      dut.release_contact_surface_mesh()->Equal(*contact_surface_mesh_W));
}

// Verifies that the constructor of DeformableRigidContactSurface
// creates the correct rotation matrices.
TYPED_TEST(DeformableRigidContactSurfaceTest, RotationMatrices) {
  using T = TypeParam;
  DeformableRigidContactSurface<T> dut = MakeDeformableRigidContactSurface<T>();
  std::unique_ptr<PolygonSurfaceMesh<T>> contact_surface_mesh_W =
      MakePolygonSurfaceMesh<T>();
  const Vector3<T>& Cz_W = contact_surface_mesh_W->face_normal(0);

  // Let R_CW = R_CWs[0] be the rotation matrix from World frame to the contact
  // frame C of the first contact polygon. It should map the contact normal
  // Cz_W to the basis vector Cz of the contact frame C.
  const Vector3<T> Cz_C = Vector3<T>::UnitZ();
  const T kEps = 4 * std::numeric_limits<T>::epsilon();
  ASSERT_EQ(dut.R_CWs().size(), 1);
  EXPECT_TRUE(CompareMatrices(dut.R_CWs()[0] * Cz_W, Cz_C, kEps));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
