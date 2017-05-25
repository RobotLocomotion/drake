#include "drake/systems/framework/vector_value.h"

#include <gtest/gtest.h>

#include "drake/common/test/is_dynamic_castable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/test_utilities/my_vector.h"

namespace drake {
namespace systems {
namespace {

using MyVector2d = MyVector<2, double>;

GTEST_TEST(VectorValueTest, Access) {
  VectorValue<double> value(BasicVector<double>::Make({1, 2, 3}));
  EXPECT_EQ(1, value.get_value()->get_value().x());
  EXPECT_EQ(2, value.get_value()->get_value().y());
  EXPECT_EQ(3, value.get_value()->get_value().z());
}

GTEST_TEST(VectorValueTest, CloneSubclass) {
  VectorValue<double> value(MyVector2d::Make(1, 2));

  // Clone of VectorValue should produce a VectorValue.
  std::unique_ptr<AbstractValue> abstract_value = value.Clone();
  const VectorValue<double>* const new_value =
      dynamic_cast<VectorValue<double>*>(abstract_value.get());
  ASSERT_TRUE(new_value != nullptr);

  // The underlying BasicVector should be a MyVector.
  const BasicVector<double>* const new_vector = new_value->get_value();
  ASSERT_TRUE(new_vector != nullptr);
  auto my_vector = dynamic_cast<const MyVector2d*>(new_vector);
  ASSERT_TRUE(my_vector != nullptr);
  EXPECT_EQ(1, my_vector->GetAtIndex(0));
  EXPECT_EQ(2, my_vector->GetAtIndex(1));
}

}  // namespace
}  // namespace systems
}  // namespace drake
