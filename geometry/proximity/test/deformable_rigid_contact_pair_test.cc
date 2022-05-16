#include "drake/geometry/proximity/deformable_rigid_contact_pair.h"

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
  const Vector3<double> nhat_W = Vector3<double>(1, 2, 3).normalized();
  // Only the unit normal is relevant to this test. Use default values for
  // the others. Use one polygon for the contact surface for simplicity.
  const std::vector<ContactPolygonData<double>> polygon_data{
      {.unit_normal = nhat_W}};

  const DeformableRigidContactPair<double> dut(
      DeformableContactSurface<double>(polygon_data),
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
