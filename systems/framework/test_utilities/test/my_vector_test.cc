#include "drake/systems/framework/test_utilities/my_vector.h"

#include <gtest/gtest.h>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/common/value.h"

namespace drake {
namespace systems {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

// Make sure all the constructors work properly.
GTEST_TEST(MyVectorTest, Construction) {
  // Default constructor leaves memory uninitialized so don't peek.
  MyVector3d default_vector;
  EXPECT_TRUE(is_dynamic_castable<BasicVector<double>>(&default_vector));
  EXPECT_EQ(default_vector.size(), 3);

  Vector4d fixed_size4(1., 2., 3., 4.);
  VectorX<double> variable_size4(fixed_size4);

  MyVector4d from_fixed_size(fixed_size4);
  EXPECT_EQ(from_fixed_size.get_value(), variable_size4);

  MyVector4d from_variable_size(variable_size4);
  EXPECT_EQ(from_variable_size.get_value(), fixed_size4);
}

// Misuse of the test utility is an abort-able infraction.
GTEST_TEST(MyVectorTest, BadSize) {
  Vector4d fixed_size4(1., 2., 3., 4.);

  DRAKE_EXPECT_THROWS_MESSAGE(
      MyVector3d(VectorX<double>(fixed_size4)),
      ".*size.*fail.*");

  // This won't compile since there is no constructor with mismatched sizes.
  // MyVector3d from_4(fixed_size4);
}

// Test the Make() method (takes a variable length arg list).
GTEST_TEST(MyVectorTest, MakeMethod) {
  auto vector3 = MyVector3d::Make(1., 2., 3.);
  EXPECT_EQ(vector3->get_value(), Vector3d(1., 2., 3.));

  auto vector4 = MyVector4d::Make(10, 20, 30, 40);
  EXPECT_EQ(vector4->get_value(), Vector4d(10., 20., 30., 40.));
}

// Tests that cloning works and check compatibility with copyable_unique_ptr
// and AbstractValue.
GTEST_TEST(MyVectorTest, Clone) {
  MyVector3d vector3(Vector3d(1., 2., 3.));
  auto clone = vector3.Clone();

  // Changing the original should not affect the clone.
  vector3[1] = 20.;
  EXPECT_EQ(vector3.get_value(), Vector3d(1., 20., 3.));

  EXPECT_EQ(clone->get_value(), Vector3d(1., 2., 3.));

  // Value<T> requires that T be copyable or cloneable.
  auto abstract3 = AbstractValue::Make(vector3);
  auto& casted3 = abstract3->get_value<MyVector3d>();
  EXPECT_EQ(casted3.get_value(), vector3.get_value());

  // copyable_unique_ptr<T> requires that T be copyable or cloneable.
  copyable_unique_ptr<MyVector2d> vec1{new MyVector2d(Vector2d(1, 2))};
  EXPECT_EQ(vec1->size(), 2);
  EXPECT_EQ(vec1->get_value(), Vector2d(1, 2));

  copyable_unique_ptr<MyVector2d> vec2(vec1);
  vec2->set_value(Vector2d(9, 10));
  EXPECT_EQ(vec1->get_value(), Vector2d(1, 2));
  EXPECT_EQ(vec2->get_value(), Vector2d(9, 10));
}

}  // namespace
}  // namespace systems
}  // namespace drake
