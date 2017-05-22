#include "drake/systems/framework/vector_value.h"

#include <gtest/gtest.h>

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

GTEST_TEST(VectorValueTest, CopyConstructor) {
  VectorValue<double> value(BasicVector<double>::Make({1, 2, 3}));

  VectorValue<double> other_value(value);
  EXPECT_EQ(1, other_value.get_value()->get_value().x());
  EXPECT_EQ(2, other_value.get_value()->get_value().y());
  EXPECT_EQ(3, other_value.get_value()->get_value().z());
}

GTEST_TEST(VectorValueTest, CopyConstructorSubclass) {
  VectorValue<double> value(MyVector2d::Make(1, 2));
  VectorValue<double> other_value(value);
  value.get_value()->set_value(Eigen::Vector2d(3, 4));

  const MyVector2d* const vector =
      dynamic_cast<const MyVector2d*>(other_value.get_value());
  ASSERT_NE(vector, nullptr);
  EXPECT_EQ(vector->get_value()(0), 1);
  EXPECT_EQ(vector->get_value()(1), 2);
}

GTEST_TEST(VectorValueTest, CopyConstructorNull) {
  VectorValue<double> value{nullptr};

  VectorValue<double> other_value(value);
  EXPECT_EQ(other_value.get_value(), nullptr);
}

GTEST_TEST(VectorValueTest, AssignmentOperator) {
  VectorValue<double> value(BasicVector<double>::Make({1, 2, 3}));
  VectorValue<double> other_value(BasicVector<double>::Make({4, 5, 6}));

  value = other_value;
  EXPECT_EQ(4, value.get_value()->get_value().x());
  EXPECT_EQ(5, value.get_value()->get_value().y());
  EXPECT_EQ(6, value.get_value()->get_value().z());
}

GTEST_TEST(VectorValueTest, AssignmentOperatorSubclass) {
  VectorValue<double> value(MyVector2d::Make(1, 2));
  VectorValue<double> other_value(BasicVector<double>::Make({5, 6}));
  other_value = value;
  value.get_value()->set_value(Eigen::Vector2d(3, 4));

  const MyVector2d* const vector =
      dynamic_cast<const MyVector2d*>(other_value.get_value());
  ASSERT_NE(vector, nullptr);
  EXPECT_EQ(vector->get_value()(0), 1);
  EXPECT_EQ(vector->get_value()(1), 2);
}

GTEST_TEST(VectorValueTest, AssignmentOperatorNull) {
  VectorValue<double> value{nullptr};
  VectorValue<double> other_value(BasicVector<double>::Make({4, 5, 6}));

  value = other_value;
  other_value = VectorValue<double>{nullptr};
  EXPECT_EQ(4, value.get_value()->get_value().x());
  EXPECT_EQ(5, value.get_value()->get_value().y());
  EXPECT_EQ(6, value.get_value()->get_value().z());
  EXPECT_EQ(other_value.get_value(), nullptr);
}

GTEST_TEST(VectorValueTest, AssignmentOperatorSelf) {
  VectorValue<double> value(BasicVector<double>::Make({4, 5, 6}));
  value = value;
  EXPECT_EQ(4, value.get_value()->get_value().x());
  EXPECT_EQ(5, value.get_value()->get_value().y());
  EXPECT_EQ(6, value.get_value()->get_value().z());
}

GTEST_TEST(VectorValueTest, ExtractBasicVector) {
  auto basic_vector = BasicVector<double>::Make({4, 5, 6});
  const BasicVector<double>* ptr = basic_vector.get();
  VectorValue<double> value(std::move(basic_vector));

  const BasicVector<double>& got = value.get_vector();
  EXPECT_EQ(&got, ptr);
  EXPECT_EQ(got.get_value(), Eigen::Vector3d(4, 5, 6));

  value.get_mutable_vector().SetAtIndex(1, 9.);
  EXPECT_EQ(got.get_value(), Eigen::Vector3d(4, 9, 6));

  auto extracted = value.release_vector();
  EXPECT_EQ(extracted.get(), ptr);

  // Check that the VectorValue is is empty now.
  EXPECT_TRUE(value.release_vector() == nullptr);
}

}  // namespace
}  // namespace systems
}  // namespace drake
