#include "drake/geometry/proximity/deformable_rigid_contact_pair.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace {

// Verifies the constructor of DeformableRigidContactPair
// that it creates the correct rotation matrices. Use `double` as a
// representation scalar type.
GTEST_TEST(DeformableRigidContactPairTest, TestConstructor) {
  const Vector3<double> nhat_W = Vector3<double>(1, 2, 3).normalized();
  // Only the unit normal is relevant to this test. Use default values for
  // the others. Use one polygon for the contact surface for simplicity.
  const std::vector<ContactPolygonData<double>> polygon_data{
      {.unit_normal = nhat_W}};
  // Arbitrary values for the sake of unit tests.
  const DeformableRigidContactPair<double> dut(
      DeformableContactSurface<double>(polygon_data), GeometryId::get_new_id(),
      3, 0.1, 0.2, 0.3);
  const math::RotationMatrix<double> expect_R_CW =
      math::RotationMatrix<double>::MakeFromOneUnitVector(nhat_W, 2)
          .transpose();
  // Use exact comparison because the same calculation was done in the
  // constructor of DeformableRigidContactPair.
  EXPECT_TRUE(CompareMatrices(dut.R_CWs[0].matrix(), expect_R_CW.matrix()));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
