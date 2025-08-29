#include "drake/systems/framework/wrapped_system.h"

#include <memory>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/unused.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
#include "drake/systems/primitives/adder.h"

namespace drake {
namespace systems {
namespace internal {
namespace {

using symbolic::Expression;

GTEST_TEST(WrappedSystemTest, Basic) {
  auto adder = std::make_shared<Adder<double>>(2, 1);
  adder->set_name("my_adder");
  WrappedSystem<double> dut(std::move(adder));
  EXPECT_EQ(dut.get_name(), "my_adder");
  EXPECT_EQ(dut.num_input_ports(), 2);
  EXPECT_EQ(dut.num_output_ports(), 1);
  EXPECT_TRUE(dynamic_cast<const Adder<double>*>(&dut.unwrap()));
}

GTEST_TEST(WrappedSystemTest, ConvertScalarType) {
  auto adder = std::make_shared<Adder<double>>(2, 1);
  adder->set_name("my_adder");
  WrappedSystem<double> dut(std::move(adder));
  EXPECT_TRUE(is_autodiffxd_convertible(dut, [&](const auto& converted) {
    EXPECT_EQ(converted.get_name(), "my_adder");
    EXPECT_EQ(converted.num_input_ports(), 2);
    EXPECT_EQ(converted.num_output_ports(), 1);
    EXPECT_NO_THROW(
        unused(dynamic_cast<const Adder<AutoDiffXd>&>(converted.unwrap())));
  }));
  EXPECT_TRUE(is_symbolic_convertible(dut, [&](const auto& converted) {
    EXPECT_EQ(converted.get_name(), "my_adder");
    EXPECT_EQ(converted.num_input_ports(), 2);
    EXPECT_EQ(converted.num_output_ports(), 1);
    EXPECT_NO_THROW(
        unused(dynamic_cast<const Adder<Expression>&>(converted.unwrap())));
  }));
}

}  // namespace
}  // namespace internal
}  // namespace systems
}  // namespace drake
