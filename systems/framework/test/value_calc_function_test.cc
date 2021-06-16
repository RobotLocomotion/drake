#include "drake/systems/framework/value_calc_function.h"

#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/systems/framework/abstract_value_cloner.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {
namespace {

using AllocateCallback = ValueCalcFunction::AllocateCallback;
using CalcCallback = ValueCalcFunction::CalcCallback;

// The permitted argument types to specify Calc are:
// - (1) `calc` is a member function pointer with an output argument
// - (2) `calc` is a member function pointer with a return value
// - (3) `calc` is a std::function with an output argument
// - (4) `calc` is a std::function with a return value
// - (5) `calc` is a generic CalcCallback

// The permitted argument types to specify Allocate are:
// - (a) `allocate` is via the default constructor
// - (b) `allocate` is via user-supplied model_value
// - (c) `allocate` is a member function pointer
// - (d) `allocate` is a generic AllocateCallback

class MyClass {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MyClass)

  MyClass() = default;

  void CalcOne(const Context<double>&, std::string* out) const {
    *out = "one";
  }

  std::string CalcTwo(const Context<double>&) const {
    return "two";
  }

  std::unique_ptr<std::string> AllocateCharlie() const {
    return std::make_unique<std::string>("charlie");
  }
};

void CalcThree(const Context<double>&, std::string* out) {
  *out = "three";
}

std::string CalcFour(const Context<double>&) {
  return "four";
}

void CalcFive(const ContextBase&, AbstractValue* out) {
  out->set_value<std::string>("five");
}

std::unique_ptr<AbstractValue> AllocateDelta() {
  return AbstractValue::Make<std::string>("delta");
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

  const std::string kBravo{"bravo"};

  const MyClass my_class_;
  const LeafContext<double> context_;
};

// Test the default constructor.
TEST_F(ValueCalcFunctionTest, DefaultCtor) {
  const ValueCalcFunction empty;
  const ValueCalcFunction empty_copy(empty);
  EXPECT_FALSE(empty_copy.is_valid());
  EXPECT_THROW(empty_copy.Allocate(), std::exception);

  auto output = AbstractValue::Make<int>(0);
  EXPECT_THROW(empty_copy.Calc(context_, output.get()), std::exception);
}

// Test the constructor when given invalid function(s).
TEST_F(ValueCalcFunctionTest, InvalidCallbacks) {
  const AllocateCallback good_allocate = internal::AbstractValueCloner(0);
  const AllocateCallback bad_allocate;
  const CalcCallback good_calc = &ValueCalcFunction::NoopCalc;
  const CalcCallback bad_calc;
  EXPECT_NO_THROW(ValueCalcFunction(good_allocate, good_calc));
  EXPECT_THROW(ValueCalcFunction(bad_allocate, good_calc), std::exception);
  EXPECT_THROW(ValueCalcFunction(good_allocate, bad_calc), std::exception);
  EXPECT_THROW(ValueCalcFunction(bad_allocate, bad_calc), std::exception);
}

TEST_F(ValueCalcFunctionTest, AlphaOne) {
  const ValueCalcFunction dut(&my_class_, &MyClass::CalcOne);
  CheckValues(dut, "", "one");
}

TEST_F(ValueCalcFunctionTest, AlphaTwo) {
  const ValueCalcFunction dut(&my_class_, &MyClass::CalcTwo);
  CheckValues(dut, "", "two");
}

TEST_F(ValueCalcFunctionTest, AlphaThree) {
  // N.B. We use {} not () to avoid the "most vexing parse".
  const ValueCalcFunction dut{std::function(CalcThree)};
  CheckValues(dut, "", "three");
}

TEST_F(ValueCalcFunctionTest, AlphaFour) {
  // N.B. We use {} not () to avoid the "most vexing parse".
  const ValueCalcFunction dut{std::function(CalcFour)};
  CheckValues(dut, "", "four");
}

// N.B. There is no AlphaFive (per ValueCalcFunction API docs).

TEST_F(ValueCalcFunctionTest, BravoTwo) {
  const ValueCalcFunction dut(&my_class_, kBravo, &MyClass::CalcTwo);
  CheckValues(dut, "bravo", "two");
}

TEST_F(ValueCalcFunctionTest, BravoThree) {
  const ValueCalcFunction dut(kBravo, std::function(CalcThree));
  CheckValues(dut, "bravo", "three");
}

TEST_F(ValueCalcFunctionTest, BravoFour) {
  const ValueCalcFunction dut(kBravo, std::function(CalcFour));
  CheckValues(dut, "bravo", "four");
}

TEST_F(ValueCalcFunctionTest, BravoFive) {
  const ValueCalcFunction dut(kBravo, &CalcFive);
  CheckValues(dut, "bravo", "five");
}

TEST_F(ValueCalcFunctionTest, CharlieOne) {
  const ValueCalcFunction dut(
      &my_class_, &MyClass::AllocateCharlie, &MyClass::CalcOne);
  CheckValues(dut, "charlie", "one");
}

TEST_F(ValueCalcFunctionTest, CharlieTwo) {
  const ValueCalcFunction dut(
      &my_class_, &MyClass::AllocateCharlie, &MyClass::CalcTwo);
  CheckValues(dut, "charlie", "two");
}

TEST_F(ValueCalcFunctionTest, CharlieThree) {
  const ValueCalcFunction dut(
      &my_class_, &MyClass::AllocateCharlie, std::function(CalcThree));
  CheckValues(dut, "charlie", "three");
}

TEST_F(ValueCalcFunctionTest, CharlieFour) {
  const ValueCalcFunction dut(
      &my_class_, &MyClass::AllocateCharlie, std::function(CalcFour));
  CheckValues(dut, "charlie", "four");
}

TEST_F(ValueCalcFunctionTest, CharlieFive) {
  const ValueCalcFunction dut(&my_class_, &MyClass::AllocateCharlie, &CalcFive);
  CheckValues(dut, "charlie", "five");
}

TEST_F(ValueCalcFunctionTest, DeltaOne) {
  const ValueCalcFunction dut(&my_class_, AllocateDelta, &MyClass::CalcOne);
  CheckValues(dut, "delta", "one");
}

TEST_F(ValueCalcFunctionTest, DeltaTwo) {
  const ValueCalcFunction dut(&my_class_, AllocateDelta, &MyClass::CalcTwo);
  CheckValues(dut, "delta", "two");
}

TEST_F(ValueCalcFunctionTest, DeltaThree) {
  const ValueCalcFunction dut(AllocateDelta, std::function(CalcThree));
  CheckValues(dut, "delta", "three");
}

TEST_F(ValueCalcFunctionTest, DeltaFour) {
  const ValueCalcFunction dut(AllocateDelta, std::function(CalcFour));
  CheckValues(dut, "delta", "four");
}

TEST_F(ValueCalcFunctionTest, DeltaFive) {
  const ValueCalcFunction dut(AllocateDelta, &CalcFive);
  CheckValues(dut, "delta", "five");
}

}  // namespace
}  // namespace systems
}  // namespace drake
