#include "drake/systems/framework/value_producer.h"

#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/abstract_value_cloner.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {
namespace {

using AllocateCallback = ValueProducer::AllocateCallback;
using CalcCallback = ValueProducer::CalcCallback;

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

class ValueProducerTest : public ::testing::Test {
 protected:
  void CheckValues(
      const ValueProducer& dut,
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

// Test the default constructor, as well as that operations on a default-
// constructed object produce suitable results (including operations on
// copies of a default-constructed object).
TEST_F(ValueProducerTest, DefaultCtor) {
  auto storage = AbstractValue::Make<int>(0);

  const ValueProducer empty;
  EXPECT_FALSE(empty.is_valid());
  DRAKE_EXPECT_THROWS_MESSAGE(
      empty.Allocate(), std::exception,
      ".* null Allocate.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      empty.Calc(context_, storage.get()), std::exception,
      ".* null Calc.*");

  const ValueProducer empty_copy(empty);
  EXPECT_FALSE(empty_copy.is_valid());
  DRAKE_EXPECT_THROWS_MESSAGE(
      empty_copy.Allocate(), std::exception,
      ".* null Allocate.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      empty_copy.Calc(context_, storage.get()), std::exception,
      ".* null Calc.*");
}

// Test the constructor when given invalid function(s).
TEST_F(ValueProducerTest, InvalidCallbacks) {
  const AllocateCallback good_allocate = internal::AbstractValueCloner(0);
  const AllocateCallback bad_allocate;
  const CalcCallback good_calc = &ValueProducer::NoopCalc;
  const CalcCallback bad_calc;
  EXPECT_NO_THROW(ValueProducer(good_allocate, good_calc));
  DRAKE_EXPECT_THROWS_MESSAGE(
      ValueProducer(bad_allocate, good_calc), std::exception,
      ".* null Allocate.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      ValueProducer(good_allocate, bad_calc), std::exception,
      ".* null Calc.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      ValueProducer(bad_allocate, bad_calc), std::exception,
      ".* null .*");
}

TEST_F(ValueProducerTest, AlphaOne) {
  const ValueProducer dut(&my_class_, &MyClass::CalcOne);
  CheckValues(dut, "", "one");
}

TEST_F(ValueProducerTest, AlphaTwo) {
  const ValueProducer dut(&my_class_, &MyClass::CalcTwo);
  CheckValues(dut, "", "two");
}

TEST_F(ValueProducerTest, AlphaThree) {
  // N.B. We use {} not () to avoid the "most vexing parse".
  const ValueProducer dut{std::function(CalcThree)};
  CheckValues(dut, "", "three");
}

TEST_F(ValueProducerTest, AlphaFour) {
  // N.B. We use {} not () to avoid the "most vexing parse".
  const ValueProducer dut{std::function(CalcFour)};
  CheckValues(dut, "", "four");
}

// N.B. There is no AlphaFive (per ValueProducer API docs).

TEST_F(ValueProducerTest, BravoTwo) {
  const ValueProducer dut(&my_class_, kBravo, &MyClass::CalcTwo);
  CheckValues(dut, "bravo", "two");
}

TEST_F(ValueProducerTest, BravoThree) {
  const ValueProducer dut(kBravo, std::function(CalcThree));
  CheckValues(dut, "bravo", "three");
}

TEST_F(ValueProducerTest, BravoFour) {
  const ValueProducer dut(kBravo, std::function(CalcFour));
  CheckValues(dut, "bravo", "four");
}

TEST_F(ValueProducerTest, BravoFive) {
  const ValueProducer dut(kBravo, &CalcFive);
  CheckValues(dut, "bravo", "five");
}

TEST_F(ValueProducerTest, CharlieOne) {
  const ValueProducer dut(
      &my_class_, &MyClass::AllocateCharlie, &MyClass::CalcOne);
  CheckValues(dut, "charlie", "one");
}

TEST_F(ValueProducerTest, CharlieTwo) {
  const ValueProducer dut(
      &my_class_, &MyClass::AllocateCharlie, &MyClass::CalcTwo);
  CheckValues(dut, "charlie", "two");
}

TEST_F(ValueProducerTest, CharlieThree) {
  const ValueProducer dut(
      &my_class_, &MyClass::AllocateCharlie, std::function(CalcThree));
  CheckValues(dut, "charlie", "three");
}

TEST_F(ValueProducerTest, CharlieFour) {
  const ValueProducer dut(
      &my_class_, &MyClass::AllocateCharlie, std::function(CalcFour));
  CheckValues(dut, "charlie", "four");
}

TEST_F(ValueProducerTest, CharlieFive) {
  const ValueProducer dut(&my_class_, &MyClass::AllocateCharlie, &CalcFive);
  CheckValues(dut, "charlie", "five");
}

TEST_F(ValueProducerTest, DeltaOne) {
  const ValueProducer dut(&my_class_, AllocateDelta, &MyClass::CalcOne);
  CheckValues(dut, "delta", "one");
}

TEST_F(ValueProducerTest, DeltaTwo) {
  const ValueProducer dut(&my_class_, AllocateDelta, &MyClass::CalcTwo);
  CheckValues(dut, "delta", "two");
}

TEST_F(ValueProducerTest, DeltaThree) {
  const ValueProducer dut(AllocateDelta, std::function(CalcThree));
  CheckValues(dut, "delta", "three");
}

TEST_F(ValueProducerTest, DeltaFour) {
  const ValueProducer dut(AllocateDelta, std::function(CalcFour));
  CheckValues(dut, "delta", "four");
}

TEST_F(ValueProducerTest, DeltaFive) {
  const ValueProducer dut(AllocateDelta, &CalcFive);
  CheckValues(dut, "delta", "five");
}

}  // namespace
}  // namespace systems
}  // namespace drake
