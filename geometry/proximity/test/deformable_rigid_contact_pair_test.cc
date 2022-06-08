#include "drake/geometry/proximity/deformable_rigid_contact_pair.h"

#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// Verifies that the constructor of DeformableRigidContactPair
// creates the correct rotation matrices. Use `double` as a
// representative scalar type.
GTEST_TEST(DeformableRigidContactPairTest, TestConstructor) {
  // Only the unit normal is relevant to this test. Use one polygon for the
  // contact surface for simplicity.
  auto mesh_W = std::make_unique<PolygonSurfaceMesh<double>>(
      std::vector<int>{3, 0, 1, 2},  // One polygon of three vertices
      std::vector<Vector3<double>>{
          Vector3<double>::UnitX(),
          Vector3<double>::UnitY(),
          Vector3<double>::UnitZ(),
      });
  // Save the memory address of the mesh data structure here before the
  // unique_ptr mesh_W is reset by std::move.
  PolygonSurfaceMesh<double>* mesh_pointer = mesh_W.get();
  ContactSurface<double> contact_surface(
      GeometryId::get_new_id(), GeometryId::get_new_id(), std::move(mesh_W),
      std::make_unique<PolygonSurfaceMeshFieldLinear<double, double>>(
          std::vector<double>{-0.1, -0.1, -0.1}, mesh_pointer,
          // Constant field values means the gradient is zero.
          std::vector<Vector3<double>>{Vector3<double>::Zero()}));

  const Vector3<double> nhat_W = contact_surface.face_normal(0);

  const DeformableRigidContactPair<double> dut(
      contact_surface, std::vector<int>{0},
      // Use these arbitrary values for simplicity.
      GeometryId::get_new_id(), 3, 0.1, 0.2, 0.3);

  // Let R_CW = R_CWs[0] be the rotation matrix from World frame to the contact
  // frame C of the first contact polygon. It should map the contact normal
  // nhat_W to the basis vector Cz of the contact frame C.
  const Vector3<double> Cz_C = Vector3<double>::UnitZ();
  const double kEps = 4 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(dut.R_CWs[0] * nhat_W, Cz_C, kEps));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
