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

template <typename DerivedA, typename DerivedB>
bool CompareMatrices(const Eigen::MatrixBase<DerivedA>& m1,
                     const Eigen::MatrixBase<DerivedB>& m2) {
  return CompareMatrices(m1, m2, 1e-14 /*threshold*/,
                         MatrixCompareType::absolute);
}

GTEST_TEST(ContactResultTest, SampleCollisionManifold) {
  TestVector p0 = TestVector::Zero(3, 1);
  TestWrench w0 = TestWrench::Zero(6, 1);

  // test on empty manifold
  SampledContactManifold<double> manifold;
  EXPECT_EQ(0, manifold.get_num_contacts());
  EXPECT_EQ(nullptr, manifold.get_ith_contact(0));
  auto d_empty = manifold.ComputeNetResponse();
  EXPECT_TRUE(CompareMatrices(d_empty.get_application_point(), p0));
  EXPECT_TRUE(CompareMatrices(d_empty.get_force(), w0));

  // Test on single force

  // Zero force
  auto zeroForce = make_unique<TestContactDetail>(p0, w0);
  manifold.AddContactDetail(move(zeroForce));
  EXPECT_EQ(1, manifold.get_num_contacts());
  auto detail = manifold.get_ith_contact(0);
  TestVector p = detail->get_application_point();
  TestWrench w = detail->get_force();
  EXPECT_TRUE(CompareMatrices(p, p0));
  EXPECT_TRUE(CompareMatrices(w, w0));

  d_empty = manifold.ComputeNetResponse();
  EXPECT_TRUE(CompareMatrices(d_empty.get_application_point(), p0));
  EXPECT_TRUE(CompareMatrices(d_empty.get_force(), w0));
}
}  // namespace test
}  // namespace rigid_body_plant
}  // namespace plants
}  // namespace systems
}  // namespace drake