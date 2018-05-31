#include "drake/systems/framework/fixed_input_port_value.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/system_base.h"

// Tests the functioning of the concrete FixedInputPortValue class and
// verifies that it properly notifies dependent input ports when its value
// changes. The latter requires some hand-crafting of a suitable Context since
// we can only use framework base objects here.

namespace drake {
namespace systems {
namespace {

// Create a ContextBase with a few input ports for us to play with.
class MyContextBase : public ContextBase {
 public:
  MyContextBase() {
    AddInputPort(InputPortIndex(0), DependencyTicket(100));
    AddInputPort(InputPortIndex(1), DependencyTicket(101));
  }
  MyContextBase(const MyContextBase&) = default;

 private:
  std::unique_ptr<ContextBase> DoCloneWithoutPointers() const final {
    return std::make_unique<MyContextBase>(*this);
  }
};

// Creates a MySystemBase and its context and wires up fixed input values
// to each of the two ports.
class FixedInputPortTest : public ::testing::Test {
 protected:
  void SetUp() override {
    port0_value_ = &context_.FixInputPort(
        InputPortIndex(0), Value<BasicVector<double>>({5.0, 6.0}));

    port1_value_ = &context_.FixInputPort(
        InputPortIndex(1), Value<std::string>("foo"));

    // TODO(sherm1) Tracker & ticket setup here.

    serial0_ = port0_value_->serial_number();
    serial1_ = port1_value_->serial_number();
  }

  MyContextBase context_;

  FixedInputPortValue* port0_value_{};
  FixedInputPortValue* port1_value_{};

  int64_t serial0_{-1}, serial1_{-1};
};

// Tests that the Context wiring is set up as we assume for the
// tests here, and that the free values are wired in.
TEST_F(FixedInputPortTest, SystemAndContext) {
  // The input port trackers should have declared the free values as
  // prerequisites, and the free values should know the input ports are
  // subscribers.
  // TODO(sherm1) Tracker wiring tests go here.

  EXPECT_EQ(&port0_value_->get_owning_context(), &context_);
  EXPECT_EQ(&port1_value_->get_owning_context(), &context_);

  EXPECT_EQ(port0_value_->serial_number(), 1);
  EXPECT_EQ(port1_value_->serial_number(), 1);
}

// Fixed values are cloned only as part of cloning a context, which
// requires changing the internal context pointer that is used for value
// change notification. The ticket and input port index remain unchanged
// in the new context. The values are copied and serial numbers unchanged.
TEST_F(FixedInputPortTest, Clone) {
  std::unique_ptr<ContextBase> new_context = context_.Clone();

  auto free0 = new_context->MaybeGetMutableFixedInputPortValue(0);
  auto free1 = new_context->MaybeGetMutableFixedInputPortValue(1);
  ASSERT_NE(free0, nullptr);
  ASSERT_NE(free1, nullptr);

  EXPECT_EQ(&free0->get_owning_context(), new_context.get());
  EXPECT_EQ(&free1->get_owning_context(), new_context.get());

  EXPECT_EQ(free0->get_vector_value<double>().get_value(),
            port0_value_->get_vector_value<double>().get_value());
  EXPECT_EQ(free1->get_value().GetValue<std::string>(),
            port1_value_->get_value().GetValue<std::string>());

  EXPECT_EQ(free0->serial_number(), port0_value_->serial_number());
  EXPECT_EQ(free1->serial_number(), port1_value_->serial_number());

  // Make sure changing a value in the old context doesn't change a value
  // in the new one, and vice versa.
  port0_value_->GetMutableVectorData<double>()->SetAtIndex(0, 99);
  EXPECT_EQ(port0_value_->get_vector_value<double>().GetAtIndex(0), 99);
  EXPECT_EQ(free0->get_vector_value<double>().GetAtIndex(0), 5);

  free1->GetMutableData()->GetMutableValue<std::string>() = "hello";
  EXPECT_EQ(free1->get_value().GetValue<std::string>(), "hello");
  EXPECT_EQ(port1_value_->get_value().GetValue<std::string>(), "foo");
}

// Test that we can access values and that doing so does not send value change
// notifications.
TEST_F(FixedInputPortTest, Access) {
  EXPECT_EQ(Vector2<double>(5, 6),
            port0_value_->get_vector_value<double>().get_value());
  EXPECT_EQ(port0_value_->serial_number(), serial0_);

  EXPECT_EQ("foo", port1_value_->get_value().GetValue<std::string>());
  EXPECT_EQ(port1_value_->serial_number(), serial1_);
}

// Tests that changes to the vector data are propagated to the input port
// that wraps it.
TEST_F(FixedInputPortTest, Mutation) {
  // Change the vector port's value.
  port0_value_->GetMutableVectorData<double>()->get_mutable_value()
      << 7, 8;
  // Check that the vector contents changed.
  EXPECT_EQ(Vector2<double>(7, 8),
            port0_value_->get_vector_value<double>().get_value());
  // Check that notifications were sent and serial number bumped.
  // TODO(sherm1) Check notifications.
  EXPECT_EQ(port0_value_->serial_number(), serial0_ + 1);

  // Change the abstract port's value.
  port1_value_->GetMutableData()->GetMutableValue<std::string>() = "bar";
  // Check that the contents changed.
  EXPECT_EQ("bar", port1_value_->get_value().GetValue<std::string>());
  // Check that notifications were sent and serial number bumped.
  // TODO(sherm1) Check notifications.
  EXPECT_EQ(port1_value_->serial_number(), serial1_ + 1);
}

}  // namespace
}  // namespace systems
}  // namespace drake
