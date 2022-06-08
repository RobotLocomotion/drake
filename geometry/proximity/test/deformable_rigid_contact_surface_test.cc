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
PolygonSurfaceMesh<T> MakeSurfaceMesh() {
  // Make a polygon mesh with a single triangle with vertices at (1, 0, 0),
  // (0, 1, 0), and (0, 0, 1).
  return PolygonSurfaceMesh<T>(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<T>>{
          Vector3<T>::UnitX(),
          Vector3<T>::UnitY(),
          Vector3<T>::UnitZ(),
      });
}

/* Builds a DeformableRigidContactSurface for testing purpose.
 The contact surface that involves a single contact polygon and a single contact
 point with a signed distance of kSignedDistance. The single contact polygon is
 contained in the tetrahedron with index kTetIndex. See constructor of
 DeformableRigidContactSurface. */
template <typename T>
DeformableRigidContactSurface<T> MakeDeformableRigidContactSurface() {
  return DeformableRigidContactSurface<T>(
      MakeSurfaceMesh<T>(), std::vector<double>{kSignedDistance},
      std::vector<int>{kTetIndex},
      std::vector<Vector4<T>>{Vector4<T>{0.1, 0.2, 0.3, 0.4}}, kRigidId,
      kDeformableId);
}

using ScalarTypes = ::testing::Types<double>;
TYPED_TEST_SUITE(DeformableRigidContactSurfaceTest, ScalarTypes);
template <typename T>
class DeformableRigidContactSurfaceTest : public ::testing::Test {};

TYPED_TEST(DeformableRigidContactSurfaceTest, DefaultConstructor) {
  using T = TypeParam;
  DeformableRigidContactSurface<T> dut;
  // Default constructed DeformableRigidContactSurface has no data.
  EXPECT_FALSE(dut.has_data());
}

// Tests the constructor and the release_data() function.
TYPED_TEST(DeformableRigidContactSurfaceTest, ConstructAndRelease) {
  using T = TypeParam;
  DeformableRigidContactSurface<T> dut = MakeDeformableRigidContactSurface<T>();

  // Verify that some data exists.
  EXPECT_TRUE(dut.has_data());

  const int kNumContactPoints = 1;
  EXPECT_EQ(dut.deformable_id(), kDeformableId);
  EXPECT_EQ(dut.rigid_id(), kRigidId);
  EXPECT_EQ(dut.num_contact_points(), kNumContactPoints);

  const auto [contact_mesh_W, penetration_distances, tetrahedron_indices,
              barycentric_centroids, R_CWs] = dut.release_data();
  ASSERT_EQ(tetrahedron_indices.size(), kNumContactPoints);
  EXPECT_EQ(tetrahedron_indices[0], kTetIndex);
  ASSERT_EQ(barycentric_centroids.size(), kNumContactPoints);
  EXPECT_EQ(barycentric_centroids[0], Vector4<T>(0.1, 0.2, 0.3, 0.4));
  ASSERT_EQ(penetration_distances.size(), kNumContactPoints);
  EXPECT_EQ(penetration_distances[0], kSignedDistance);
  EXPECT_TRUE(contact_mesh_W.Equal(MakeSurfaceMesh<T>()));

  // Verify that the data has been released.
  EXPECT_FALSE(dut.has_data());
}

// Verifies that the constructor of DeformableRigidContactSurface
// creates the correct rotation matrices.
TYPED_TEST(DeformableRigidContactSurfaceTest, RotationMatrices) {
  using T = TypeParam;
  DeformableRigidContactSurface<T> dut = MakeDeformableRigidContactSurface<T>();
  const auto data = dut.release_data();
  const PolygonSurfaceMesh<double>& contact_mesh_W = std::get<0>(data);
  const std::vector<math::RotationMatrixd>& R_CWs = std::get<4>(data);
  const Vector3<T>& Cz_W = contact_mesh_W.face_normal(0);

  // Let R_CW = R_CWs[0] be the rotation matrix from World frame to the contact
  // frame C of the first contact polygon. It should map the contact normal
  // Cz_W to the basis vector Cz of the contact frame C.
  const Vector3<T> Cz_C = Vector3<T>::UnitZ();
  const T kEps = 4 * std::numeric_limits<T>::epsilon();
  ASSERT_EQ(R_CWs.size(), 1);
  EXPECT_TRUE(CompareMatrices(R_CWs[0] * Cz_W, Cz_C, kEps));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
