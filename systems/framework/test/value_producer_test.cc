#include "drake/systems/framework/value_producer.h"

#include <stdexcept>
#include <string>

#include <gmock/gmock.h>
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
// - (1) `calc` is a member function pointer with an output argument.
// - (2) `calc` is a member function pointer with a return value.
// - (3) `calc` is a std::function with an output argument.
// - (4) `calc` is a std::function with a return value.
// - (5) `calc` is a generic CalcCallback.

// The permitted argument types to specify Allocate are:
// - (a) `allocate` is via the default constructor.
// - (b) `allocate` is via user-supplied model_value.
// - (c) `allocate` is a member function pointer.
// - (d) `allocate` is a generic AllocateCallback.

// In the below, we'll declare functions and/or literal values that cover all
// of these options, so that we can easily mix and match them in test cases.
// All combinations are tested below except for (5a) which doesn't make sense.

// For readability, we'll spell out (1) as "one" and (a) as "alpha", etc.

class MyClass {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MyClass)

  MyClass() = default;

  // &MyClass::CalcOne is a valid argument for series (1).
  void CalcOne(const Context<double>&, std::string* out) const {
    *out = "one";
  }

  // &MyClass::CalcTwo is a valid argument for series (2).
  std::string CalcTwo(const Context<double>&) const {
    return "two";
  }

  // &MyClass::AllocateCharlie is a valid argument for series (c).
  std::unique_ptr<std::string> AllocateCharlie() const {
    return std::make_unique<std::string>("charlie");
  }
};

// std::function(&CalcThree) is a valid argument for series (3).
//
// Note that `&CalcThree` on its own would fail template argument deduction,
// so for function pointers we must help the compiler by spelling out the
// std::function conversion.  In the typical case a user will write a lambda
// lambda instead of a function pointer, so it's not a big deal in practice.
void CalcThree(const Context<double>&, std::string* out) {
  *out = "three";
}

// std::function(&CalcFour) is a valid argument for series (4).
//
// Note that `&CalcFour` on its own would fail template argument deduction,
// so for function pointers we must help the compiler by spelling out the
// std::function conversion.  In the typical case a user will write a lambda
// instead of a function pointer, so it's not a big deal in practice.
std::string CalcFour(const Context<double>&) {
  return "four";
}

// &CalcFive is a valid argument for series (5).
void CalcFive(const ContextBase&, AbstractValue* out) {
  out->set_value<std::string>("five");
}

// &AllocateDelta is a valid argument for series (d).
std::unique_ptr<AbstractValue> AllocateDelta() {
  return AbstractValue::Make<std::string>("delta");
}

// kBadCalcOne is an null argument for series (1).
void (MyClass::* const kBadCalcOne)(const Context<double>&, std::string*) const
    = nullptr;

// kBadCalcTwo is an null argument for series (2).
std::string (MyClass::* const kBadCalcTwo)(const Context<double>&) const
    = nullptr;

// std::function(kBadCalcThree) is an null argument for series (3).
void (* const kBadCalcThree)(const Context<double>&, std::string*)
    = nullptr;

// std::function(kBadCalcFour) is an null argument for series (4).
std::string (* const kBadCalcFour)(const Context<double>&)
    = nullptr;

// kBadCalcFive is an null argument for series (5).
void (* const kBadCalcFive)(const ContextBase&, AbstractValue*)
    = nullptr;

// kBadAllocateCharlie is an null argument for series (c).
std::unique_ptr<std::string> (MyClass::* const kBadAllocateCharlie)() const
    = nullptr;

// kBadAllocateDelta is an null argument for series (d).
std::unique_ptr<AbstractValue> (* const kBadAllocateDelta)()
    = nullptr;

// kBadInstance provides a typed null when a SomeInstance is needed.
const MyClass* const kBadInstance = nullptr;

class ValueProducerTest : public ::testing::Test {
 protected:
  // Given a device under test, check that its Allocate and Calc produce the
  // given strings.
  void CheckValues(
      const ValueProducer& dut,
      std::string_view expected_allocate_value,
      std::string_view expected_calc_value) {
    EXPECT_TRUE(dut.is_valid());
    std::unique_ptr<AbstractValue> output = dut.Allocate();
    ASSERT_NE(output, nullptr);
    EXPECT_EQ(output->get_value<std::string>(), expected_allocate_value);
    dut.Calc(context_, output.get());
    EXPECT_EQ(output->get_value<std::string>(), expected_calc_value);
  }

  // kBravo is a valid argument for series (b).
  const std::string kBravo{"bravo"};

  // Dummy values for convenience.
  const MyClass my_class_{};
  const LeafContext<double> context_{};
};

// Test the default constructor, as well as that operations on a default-
// constructed object produce suitable results (including operations on
// copies of a default-constructed object).
TEST_F(ValueProducerTest, DefaultCtor) {
  auto storage = AbstractValue::Make<int>(0);

  const ValueProducer empty;
  EXPECT_FALSE(empty.is_valid());
  DRAKE_EXPECT_THROWS_MESSAGE(
      empty.Allocate(),
      ".* null Allocate.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      empty.Calc(context_, storage.get()),
      ".* null Calc.*");

  const ValueProducer empty_copy(empty);
  EXPECT_FALSE(empty_copy.is_valid());
  DRAKE_EXPECT_THROWS_MESSAGE(
      empty_copy.Allocate(),
      ".* null Allocate.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      empty_copy.Calc(context_, storage.get()),
      ".* null Calc.*");
}

// Check that the given constructor arguments throw a null pointer exception.
#define DRAKE_CHECK_CTOR_NULL(constructor_args) \
  DRAKE_EXPECT_THROWS_MESSAGE(ValueProducer constructor_args, \
      ".* null .*");

TEST_F(ValueProducerTest, AlphaOne) {
  const ValueProducer dut(&my_class_, &MyClass::CalcOne);
  CheckValues(dut, "", "one");
  DRAKE_CHECK_CTOR_NULL((kBadInstance, &MyClass::CalcOne));
  DRAKE_CHECK_CTOR_NULL((&my_class_, kBadCalcOne));
}

TEST_F(ValueProducerTest, AlphaTwo) {
  const ValueProducer dut(&my_class_, &MyClass::CalcTwo);
  CheckValues(dut, "", "two");
  DRAKE_CHECK_CTOR_NULL((kBadInstance, &MyClass::CalcTwo));
  DRAKE_CHECK_CTOR_NULL((&my_class_, kBadCalcTwo));
}

TEST_F(ValueProducerTest, AlphaThree) {
  // N.B. We use {} not () to avoid the "most vexing parse".
  const ValueProducer dut{std::function(CalcThree)};
  CheckValues(dut, "", "three");
  DRAKE_CHECK_CTOR_NULL((std::function(kBadCalcThree)));
}

TEST_F(ValueProducerTest, AlphaFour) {
  // N.B. We use {} not () to avoid the "most vexing parse".
  const ValueProducer dut{std::function(CalcFour)};
  CheckValues(dut, "", "four");
  DRAKE_CHECK_CTOR_NULL((std::function(kBadCalcFour)));
}

// N.B. Per ValueProducer API docs, there is no AlphaFive (aka (5a)).

TEST_F(ValueProducerTest, BravoTwo) {
  const ValueProducer dut(&my_class_, kBravo, &MyClass::CalcTwo);
  CheckValues(dut, "bravo", "two");
  DRAKE_CHECK_CTOR_NULL((kBadInstance, kBravo, &MyClass::CalcTwo));
  DRAKE_CHECK_CTOR_NULL((&my_class_, kBravo, kBadCalcTwo));
}

TEST_F(ValueProducerTest, BravoThree) {
  const ValueProducer dut(kBravo, std::function(CalcThree));
  CheckValues(dut, "bravo", "three");
  DRAKE_CHECK_CTOR_NULL((kBravo, std::function(kBadCalcThree)));
}

TEST_F(ValueProducerTest, BravoFour) {
  const ValueProducer dut(kBravo, std::function(CalcFour));
  CheckValues(dut, "bravo", "four");
  DRAKE_CHECK_CTOR_NULL((kBravo, std::function(kBadCalcFour)));
}

TEST_F(ValueProducerTest, BravoFive) {
  const ValueProducer dut(kBravo, &CalcFive);
  CheckValues(dut, "bravo", "five");
  DRAKE_CHECK_CTOR_NULL((kBravo, kBadCalcFive));
}

TEST_F(ValueProducerTest, CharlieOne) {
  const ValueProducer dut(
      &my_class_, &MyClass::AllocateCharlie, &MyClass::CalcOne);
  CheckValues(dut, "charlie", "one");
  DRAKE_CHECK_CTOR_NULL((
      kBadInstance, &MyClass::AllocateCharlie, &MyClass::CalcOne));
  DRAKE_CHECK_CTOR_NULL((
      &my_class_, kBadAllocateCharlie, &MyClass::CalcOne));
  DRAKE_CHECK_CTOR_NULL((
      &my_class_, &MyClass::AllocateCharlie, kBadCalcOne));
}

TEST_F(ValueProducerTest, CharlieTwo) {
  const ValueProducer dut(
      &my_class_, &MyClass::AllocateCharlie, &MyClass::CalcTwo);
  CheckValues(dut, "charlie", "two");
  DRAKE_CHECK_CTOR_NULL((
      kBadInstance, &MyClass::AllocateCharlie, &MyClass::CalcTwo));
  DRAKE_CHECK_CTOR_NULL((
      &my_class_, kBadAllocateCharlie, &MyClass::CalcTwo));
  DRAKE_CHECK_CTOR_NULL((
      &my_class_, &MyClass::AllocateCharlie, kBadCalcTwo));
}

TEST_F(ValueProducerTest, CharlieThree) {
  const ValueProducer dut(
      &my_class_, &MyClass::AllocateCharlie, std::function(CalcThree));
  CheckValues(dut, "charlie", "three");
  DRAKE_CHECK_CTOR_NULL((
      kBadInstance, &MyClass::AllocateCharlie, std::function(CalcThree)));
  DRAKE_CHECK_CTOR_NULL((
      &my_class_, kBadAllocateCharlie, std::function(CalcThree)));
  DRAKE_CHECK_CTOR_NULL((
      &my_class_, &MyClass::AllocateCharlie, std::function(kBadCalcThree)));
}

TEST_F(ValueProducerTest, CharlieFour) {
  const ValueProducer dut(
      &my_class_, &MyClass::AllocateCharlie, std::function(CalcFour));
  CheckValues(dut, "charlie", "four");
  DRAKE_CHECK_CTOR_NULL((
      kBadInstance, &MyClass::AllocateCharlie, std::function(CalcFour)));
  DRAKE_CHECK_CTOR_NULL((
      &my_class_, kBadAllocateCharlie, std::function(CalcFour)));
  DRAKE_CHECK_CTOR_NULL((
      &my_class_, &MyClass::AllocateCharlie, std::function(kBadCalcFour)));
}

TEST_F(ValueProducerTest, CharlieFive) {
  const ValueProducer dut(&my_class_, &MyClass::AllocateCharlie, &CalcFive);
  CheckValues(dut, "charlie", "five");
  DRAKE_CHECK_CTOR_NULL((kBadInstance, &MyClass::AllocateCharlie, &CalcFive));
  DRAKE_CHECK_CTOR_NULL((&my_class_, kBadAllocateCharlie, &CalcFive));
  DRAKE_CHECK_CTOR_NULL((&my_class_, &MyClass::AllocateCharlie, kBadCalcFive));
}

TEST_F(ValueProducerTest, DeltaOne) {
  const ValueProducer dut(&my_class_, AllocateDelta, &MyClass::CalcOne);
  CheckValues(dut, "delta", "one");
  DRAKE_CHECK_CTOR_NULL((kBadInstance, AllocateDelta, &MyClass::CalcOne));
  DRAKE_CHECK_CTOR_NULL((&my_class_, kBadAllocateDelta, &MyClass::CalcOne));
  DRAKE_CHECK_CTOR_NULL((&my_class_, AllocateDelta, kBadCalcOne));
}

TEST_F(ValueProducerTest, DeltaTwo) {
  const ValueProducer dut(&my_class_, AllocateDelta, &MyClass::CalcTwo);
  CheckValues(dut, "delta", "two");
  DRAKE_CHECK_CTOR_NULL((kBadInstance, AllocateDelta, &MyClass::CalcTwo));
  DRAKE_CHECK_CTOR_NULL((&my_class_, kBadAllocateDelta, &MyClass::CalcTwo));
  DRAKE_CHECK_CTOR_NULL((&my_class_, AllocateDelta, kBadCalcTwo));
}

TEST_F(ValueProducerTest, DeltaThree) {
  const ValueProducer dut(AllocateDelta, std::function(CalcThree));
  CheckValues(dut, "delta", "three");
  DRAKE_CHECK_CTOR_NULL((kBadAllocateDelta, std::function(CalcThree)));
  DRAKE_CHECK_CTOR_NULL((AllocateDelta, std::function(kBadCalcThree)));
}

TEST_F(ValueProducerTest, DeltaFour) {
  const ValueProducer dut(AllocateDelta, std::function(CalcFour));
  CheckValues(dut, "delta", "four");
  DRAKE_CHECK_CTOR_NULL((kBadAllocateDelta, std::function(CalcFour)));
  DRAKE_CHECK_CTOR_NULL((AllocateDelta, std::function(kBadCalcFour)));
}

TEST_F(ValueProducerTest, DeltaFive) {
  const ValueProducer dut(AllocateDelta, &CalcFive);
  CheckValues(dut, "delta", "five");
  DRAKE_CHECK_CTOR_NULL((kBadAllocateDelta, &CalcFive));
  DRAKE_CHECK_CTOR_NULL((AllocateDelta, kBadCalcFive));
}

#undef DRAKE_CHECK_CTOR_NULL

// The below example is repeated in the documentation; keep it in sync.
TEST_F(ValueProducerTest, DocumentationExample1) {
  // This line is omitted from the documentation, for brevity.
  using T = double;

  // Documentation example begins here.
  std::function calc = [](const Context<T>& context, std::string* output) {
    *output = std::to_string(context.get_time());
  };
  ValueProducer producer(calc);
  std::unique_ptr<AbstractValue> storage = producer.Allocate();
  const LeafContext<T> context;
  producer.Calc(context, storage.get());
  EXPECT_THAT(storage->get_value<std::string>(), ::testing::StartsWith("0.0"));
}

// The below example is repeated in the documentation; keep it in sync.
TEST_F(ValueProducerTest, DocumentationExample2) {
  // This line is omitted from the documentation, for brevity.
  using T = double;

  // Documentation example begins here.  We use a different class name vs the
  // documentation, to avoid shadowing warnings.
  class MyClazz {
   public:  // This line is omitted from the documentation, for brevity.
    void MyCalc(const Context<T>& context, std::string* output) const {
      *output = std::to_string(context.get_time());
    }
  };
  MyClazz my_class;
  ValueProducer foo = ValueProducer(&my_class, &MyClazz::MyCalc);
}

// The below example is repeated in the documentation; keep it in sync.
TEST_F(ValueProducerTest, DocumentationExample3) {
  // We use a different class name vs the documentation, to avoid shadowing
  // warnings.
  class MyClazz {
   public:  // This line is omitted from the documentation, for brevity.
    double MyCalc(const Context<double>& context) const {
      return context.get_time();
    }
  };
  MyClazz my_class;
  ValueProducer foo = ValueProducer(&my_class, &MyClazz::MyCalc);
}

// The below example is repeated in the documentation; keep it in sync.
TEST_F(ValueProducerTest, DocumentationExample4) {
  // This line is omitted from the documentation, for brevity.
  using T = double;

  // Documentation example begins here.  We use a different class name vs the
  // documentation, to avoid shadowing warnings.
  class MyClazz {
   public:  // This line is omitted from the documentation, for brevity.
    void MyCalc(const Context<T>& context, BasicVector<T>* output) const {
      output->get_mutable_value()[0] = context.get_time();
    }
  };
  MyClazz my_class;
  BasicVector<T> model_value(1);
  ValueProducer foo = ValueProducer(
      &my_class, model_value, &MyClazz::MyCalc);
}

}  // namespace
}  // namespace systems
}  // namespace drake
