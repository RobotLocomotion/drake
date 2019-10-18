#include "drake/systems/framework/model_values.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/test_utilities/my_vector.h"

namespace drake {
namespace systems {
namespace internal {
namespace {

using symbolic::Expression;

/// Tests the AbstractValue basics.
GTEST_TEST(ModelValuesTest, AbstractValueTest) {
  // The "device under test".
  ModelValues dut;

  // Empty.
  EXPECT_EQ(dut.size(), 0);

  // [0] is nullptr; [1] is 11.
  dut.AddModel(1, std::make_unique<Value<int>>(11));
  EXPECT_EQ(dut.size(), 2);
  EXPECT_EQ(dut.CloneModel(0).get(), nullptr);
  auto abstract_value = dut.CloneModel(1);
  ASSERT_NE(abstract_value.get(), nullptr);
  EXPECT_EQ(abstract_value->get_value<int>(), 11);

  // Repeated clones work fine.
  ASSERT_NE(dut.CloneModel(1).get(), nullptr);

  // Expand enough to likely cause a vector reallocation.
  // [0] is nullptr; [1] is 11; [99] is 999.
  dut.AddModel(99, std::make_unique<Value<int>>(999));
  EXPECT_EQ(dut.size(), 100);
  EXPECT_EQ(dut.CloneModel(0).get(), nullptr);
  EXPECT_EQ(dut.CloneModel(1)->get_value<int>(), 11);
  EXPECT_EQ(dut.CloneModel(50).get(), nullptr);
  EXPECT_EQ(dut.CloneModel(99)->get_value<int>(), 999);

  // Unknown indices are nullptr.
  EXPECT_EQ(dut.CloneModel(1000000).get(), nullptr);

  // Test CloneAllModels().
  auto cloned_vec = dut.CloneAllModels();
  EXPECT_EQ(cloned_vec.size(), 100);
  for (int i = 0; i < 100; ++i) {
    if (i == 1) {
      EXPECT_EQ(cloned_vec[i]->get_value<int>(), 11);
    } else if (i == 99) {
      EXPECT_EQ(cloned_vec[i]->get_value<int>(), 999);
    } else {
      EXPECT_EQ(cloned_vec[i].get(), nullptr);
    }
  }
}

/// Tests the BasicVector<T> sugar.
GTEST_TEST(ModelValuesTest, VectorValueTest) {
  // The "device under test".
  ModelValues dut;

  // [1] is Value<int>(11).
  // [3] is BasicVector<double>(33, 34).
  // [5] is MyVector2d<double>(55, 56).
  dut.AddModel(1, std::make_unique<Value<int>>(11));
  dut.AddVectorModel(3, BasicVector<double>::Make({33, 34}));
  dut.AddVectorModel<double>(5, MyVector2d::Make(55, 56));

  // Empty model values are still nullptr, even with vector sugar.
  EXPECT_EQ(dut.CloneVectorModel<double>(0).get(), nullptr);
  EXPECT_EQ(dut.CloneVectorModel<double>(2).get(), nullptr);
  EXPECT_EQ(dut.CloneVectorModel<double>(4).get(), nullptr);

  // The AbstractValue is still okay.
  EXPECT_EQ(dut.CloneModel(1)->get_value<int>(), 11);

  // Some BasicVector access patterns are okay.
  EXPECT_EQ(dut.CloneVectorModel<double>(3)->size(), 2);
  EXPECT_EQ(dut.CloneVectorModel<double>(3)->GetAtIndex(0), 33);
  EXPECT_EQ(dut.CloneVectorModel<double>(3)->GetAtIndex(1), 34);
  EXPECT_EQ(dut.CloneVectorModel<double>(5)->size(), 2);
  EXPECT_EQ(dut.CloneVectorModel<double>(5)->GetAtIndex(0), 55);
  EXPECT_EQ(dut.CloneVectorModel<double>(5)->GetAtIndex(1), 56);

  // The subclass comes through okay.
  auto basic_vector = dut.CloneVectorModel<double>(5);
  MyVector2d* const my_vector = dynamic_cast<MyVector2d*>(basic_vector.get());
  ASSERT_NE(my_vector, nullptr);

  // Various wrong access patterns throw.
  EXPECT_THROW(dut.CloneVectorModel<double>(1), std::exception);
  EXPECT_THROW(dut.CloneVectorModel<AutoDiffXd>(3), std::exception);
  EXPECT_THROW(dut.CloneVectorModel<Expression>(5), std::exception);

  // Unknown indices are nullptr, even with vector sugar.
  EXPECT_EQ(dut.CloneVectorModel<double>(1000000).get(), nullptr);
}

/// Tests the details of nullptr handling.
GTEST_TEST(ModelValuesTest, NullTest) {
  // The "device under test".
  ModelValues dut;

  // Add different flavors of nullptr.
  dut.AddModel(0, nullptr);
  EXPECT_EQ(dut.size(), 1);
  dut.AddVectorModel<double>(1, nullptr);
  EXPECT_EQ(dut.size(), 2);
  dut.AddVectorModel<AutoDiffXd>(2, nullptr);
  EXPECT_EQ(dut.size(), 3);
  dut.AddVectorModel<Expression>(3, nullptr);
  EXPECT_EQ(dut.size(), 4);

  // Clone using the AbstractValue base type.
  EXPECT_EQ(dut.CloneModel(0).get(), nullptr);
  EXPECT_EQ(dut.CloneModel(1).get(), nullptr);
  EXPECT_EQ(dut.CloneModel(2).get(), nullptr);
  EXPECT_EQ(dut.CloneModel(3).get(), nullptr);

  // Clone as VectorValue (yielding a nullptr BasicVector).
  EXPECT_EQ(dut.CloneVectorModel<double>(0), nullptr);
  EXPECT_EQ(dut.CloneVectorModel<double>(1), nullptr);
  EXPECT_EQ(dut.CloneVectorModel<AutoDiffXd>(2), nullptr);
  EXPECT_EQ(dut.CloneVectorModel<Expression>(3), nullptr);
}

}  // namespace
}  // namespace internal
}  // namespace systems
}  // namespace drake
