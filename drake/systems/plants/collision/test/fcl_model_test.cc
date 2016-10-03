#include "drake/systems/plants/collision/fcl_model.h"

#include "fcl/fcl.h"
#include "gtest/gtest.h"

namespace DrakeCollision {
namespace {

// TODO(jamiesnape): Test the model.

GTEST_TEST(FCLModelTest, Test) {
  using namespace fcl;

  const Vector3<double> v1 = Vector3<double>(1.0, 2.0, 3.0);
  const Vector3<double> v2 = Vector3<double>(3.0, 4.0, 5.0);
  const Vector3<double> v3 = Vector3<double>(-2.0, 4.0, -2.0);

  EXPECT_TRUE(v1.cross(v2).isApprox(v3));
  EXPECT_DOUBLE_EQ(26.0, v1.dot(v2));
}

}  // namespace
}  // namespace DrakeCollision
