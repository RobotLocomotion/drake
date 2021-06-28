#include "drake/systems/framework/value_calc_function.h"

#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/abstract_value_cloner.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {
namespace {

using AllocateCallback = ValueCalcFunction::AllocateCallback;
using CalcCallback = ValueCalcFunction::CalcCallback;

std::unique_ptr<AbstractValue> AllocateFoo() {
  return AbstractValue::Make<std::string>("foo");
}

void CalcBar(const ContextBase&, AbstractValue* out) {
  out->set_value<std::string>("bar");
}

class ValueCalcFunctionTest : public ::testing::Test {
 protected:
  void CheckValues(
      const ValueCalcFunction& dut,
      std::string_view expected_allocate_value,
      std::string_view expected_calc_value) {
    EXPECT_TRUE(dut.is_valid());
    std::unique_ptr<AbstractValue> output = dut.Allocate();
    EXPECT_EQ(output->get_value<std::string>(), expected_allocate_value);
    dut.Calc(context_, output.get());
    EXPECT_EQ(output->get_value<std::string>(), expected_calc_value);
  }

  const LeafContext<double> context_;
};

// Test the default constructor, as well as that operations on a default-
// constructed object produce suitable results (including operations on
// copies of a default-constructed object).
TEST_F(ValueCalcFunctionTest, DefaultCtor) {
  auto storage = AbstractValue::Make<int>(0);

  const ValueCalcFunction empty;
  EXPECT_FALSE(empty.is_valid());
  DRAKE_EXPECT_THROWS_MESSAGE(
      empty.Allocate(), std::exception,
      ".* null Allocate.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      empty.Calc(context_, storage.get()), std::exception,
      ".* null Calc.*");

  const ValueCalcFunction empty_copy(empty);
  EXPECT_FALSE(empty_copy.is_valid());
  DRAKE_EXPECT_THROWS_MESSAGE(
      empty_copy.Allocate(), std::exception,
      ".* null Allocate.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      empty_copy.Calc(context_, storage.get()), std::exception,
      ".* null Calc.*");
}

// Test the constructor when given invalid function(s).
TEST_F(ValueCalcFunctionTest, InvalidCallbacks) {
  const AllocateCallback good_allocate = internal::AbstractValueCloner(0);
  const AllocateCallback bad_allocate;
  const CalcCallback good_calc = &ValueCalcFunction::NoopCalc;
  const CalcCallback bad_calc;
  EXPECT_NO_THROW(ValueCalcFunction(good_allocate, good_calc));
  DRAKE_EXPECT_THROWS_MESSAGE(
      ValueCalcFunction(bad_allocate, good_calc), std::exception,
      ".* null Allocate.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      ValueCalcFunction(good_allocate, bad_calc), std::exception,
      ".* null Calc.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      ValueCalcFunction(bad_allocate, bad_calc), std::exception,
      ".* null .*");
}

// Test the constructor when given plain functions.
TEST_F(ValueCalcFunctionTest, PlainFunction) {
  const ValueCalcFunction dut(&AllocateFoo, &CalcBar);
  CheckValues(dut, "foo", "bar");
}

}  // namespace
}  // namespace systems
}  // namespace drake
