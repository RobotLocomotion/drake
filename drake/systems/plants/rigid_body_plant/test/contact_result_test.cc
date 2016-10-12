#include <iostream>
#include <memory>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/plants/rigid_body_plant/contact_detail.h"
#include "drake/systems/plants/rigid_body_plant/sampled_contact_manifold.h"

namespace drake {
namespace systems {
namespace plants {
namespace rigid_body_plant {
namespace test {

using std::unique_ptr;
using std::make_unique;
using std::move;

typedef ContactDetail<double> TestContactDetail;
typedef Vector3<double> TestVector;
typedef WrenchVector<double> TestWrench;

GTEST_TEST(ContactResultTest, SampleCollisionManifold) {
  TestVector p0 = TestVector::Zero(3, 1);
  TestWrench w0 = TestWrench::Zero(6, 1);

  // test on empty manifold
  SampledContactManifold<double> manifold;
  EXPECT_EQ(0, manifold.get_num_contacts());
  EXPECT_EQ(nullptr, manifold.get_ith_contact(0));
  auto d_empty = manifold.ComputeNetResponse();
  EXPECT_TRUE(CompareMatrices(d_empty.get_application_point(),
                              p0, 1e-14,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(d_empty.get_force(),
                              w0, 1e-14,
                              MatrixCompareType::absolute));

}
}  // namespace test
}  // namespace rigid_body_plant
}  // namespace plants
}  // namespace systems
}  // namespace drake