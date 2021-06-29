#include "drake/systems/framework/abstract_value_cloner.h"

#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace {

// Tests AbstractValueCloner on a copyable type.
GTEST_TEST(AbstractValueClonerTest, Copyable) {
  const internal::AbstractValueCloner dut(std::string("foo"));
  const internal::AbstractValueCloner dut_copy(dut);
  std::unique_ptr<AbstractValue> new_value = dut_copy();
  EXPECT_EQ(new_value->get_value<std::string>(), "foo");
}

// Tests AbstractValueCloner on a non-copyable, cloneable type.
GTEST_TEST(AbstractValueClonerTest, Cloneable) {
  static_assert(is_cloneable<BasicVector<double>>::value, "");
  const internal::AbstractValueCloner dut(BasicVector<double>(2));
  const internal::AbstractValueCloner dut_copy(dut);
  std::unique_ptr<AbstractValue> new_value = dut_copy();
  EXPECT_EQ(new_value->get_value<BasicVector<double>>().size(), 2);
}

}  // namespace
}  // namespace systems
}  // namespace drake
