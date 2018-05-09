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

// This is so we can use the contained dependency graph for notifications.
class MyContextBase : public ContextBase {
 public:
  MyContextBase() {}
  MyContextBase(const MyContextBase&) = default;
  using ContextBase::get_mutable_dependency_graph;
 private:
  std::unique_ptr<ContextBase> DoCloneWithoutPointers() const final {
    return std::unique_ptr<ContextBase>(new MyContextBase(*this));
  }
};

// This is so we can declare a couple of input ports that will have
// dependency trackers assigned in the context.
class MySystemBase : public SystemBase {
 public:
  // Declare some input ports that we can make fixed. Port 0 is a
  // 2-element vector-valued port, port 1 is abstract valued.
  MySystemBase() {
    CreateInputPort(std::make_unique<InputPortBase>(
        InputPortIndex(0), kVectorValued, 2, nullopt, this));
    CreateInputPort(std::make_unique<InputPortBase>(
        InputPortIndex(1), kAbstractValued, 0, nullopt, this));
  }

 private:
  std::unique_ptr<ContextBase> DoMakeContext() const final {
    return std::make_unique<MyContextBase>();
  }

  void DoValidateAllocatedContext(const ContextBase&) const final {}
  void DoCheckValidContext(const ContextBase&) const final {}
};

// Creates a MySystemBase and its context and wires up fixed input values
// to each of the two ports.
class FixedInputPortTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::unique_ptr<BasicVector<double>> vec(new BasicVector<double>(2));
    vec->get_mutable_value() << 5, 6;
    port0_value_ = &context_.FixInputPort(
        InputPortIndex(0),
        std::make_unique<Value<BasicVector<double>>>(std::move(vec)));

    auto value = std::make_unique<Value<std::string>>("foo");
    port1_value_ = &context_.FixInputPort(InputPortIndex(1), std::move(value));

    // TODO(sherm1) Tracker & ticket setup here.

    serial0_ = port0_value_->serial_number();
    serial1_ = port1_value_->serial_number();
  }

  // Create a System and a Context to match.
  MySystemBase system_;
  std::unique_ptr<ContextBase> context_base_ = system_.AllocateContext();
  MyContextBase& context_ = dynamic_cast<MyContextBase&>(*context_base_);

  FixedInputPortValue* port0_value_{};
  FixedInputPortValue* port1_value_{};

  int64_t serial0_{-1}, serial1_{-1};
};

// Tests that the System and Context wiring is set up as we assume for the
// tests here, and that the free values are wired in.
TEST_F(FixedInputPortTest, SystemAndContext) {
  // The input port trackers should have declared the free values as
  // prerequisites, and the free values should know the input ports are
  // subscribers.
  // TODO(sherm1) Tracker wiring tests go here.

  EXPECT_EQ(port0_value_->input_port_index(), InputPortIndex(0));
  EXPECT_EQ(port1_value_->input_port_index(), InputPortIndex(1));

  EXPECT_EQ(&port0_value_->get_owning_context(), context_base_.get());
  EXPECT_EQ(&port1_value_->get_owning_context(), context_base_.get());

  EXPECT_EQ(port0_value_->serial_number(), 1);
  EXPECT_EQ(port1_value_->serial_number(), 1);
}

// Fixed values are cloned only as part of cloning a context, which
// requires changing the internal context pointer that is used for value
// change notification. The ticket and input port index remain unchanged
// in the new context. The values are copied and serial numbers unchanged.
TEST_F(FixedInputPortTest, Clone) {
  std::unique_ptr<ContextBase> new_context = context_.Clone();

  auto free0 = new_context->MaybeGetFixedInputPortValue(0);
  auto free1 = new_context->MaybeGetFixedInputPortValue(1);
  ASSERT_NE(free0, nullptr);
  ASSERT_NE(free1, nullptr);

  EXPECT_EQ(&free0->get_owning_context(), new_context.get());
  EXPECT_EQ(&free1->get_owning_context(), new_context.get());

  EXPECT_EQ(free0->template get_vector_value<double>().get_value(),
            port0_value_->template get_vector_value<double>().get_value());
  EXPECT_EQ(free1->get_value().GetValue<std::string>(),
            port1_value_->get_value().GetValue<std::string>());

  EXPECT_EQ(free0->serial_number(), port0_value_->serial_number());
  EXPECT_EQ(free1->serial_number(), port1_value_->serial_number());
}

// Test that we can access values and that doing so does not send value change
// notifications.
TEST_F(FixedInputPortTest, Access) {
  EXPECT_EQ(Vector2<double>(5, 6),
            port0_value_->template get_vector_value<double>().get_value());
  EXPECT_EQ(port0_value_->serial_number(), serial0_);

  EXPECT_EQ("foo", port1_value_->get_value().GetValue<std::string>());
  EXPECT_EQ(port1_value_->serial_number(), serial1_);
}

// Tests that changes to the vector data are propagated to the input port
// that wraps it.
TEST_F(FixedInputPortTest, Mutation) {
  // Change the vector port's value.
  port0_value_->template GetMutableVectorData<double>()->get_mutable_value()
      << 7, 8;
  // Check that the vector contents changed.
  EXPECT_EQ(Vector2<double>(7, 8),
            port0_value_->template get_vector_value<double>().get_value());
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
