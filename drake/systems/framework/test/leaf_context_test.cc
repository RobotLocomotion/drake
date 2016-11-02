#include "drake/systems/framework/leaf_context.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/framework/test_utilities/pack_value.h"

namespace drake {
namespace systems {

constexpr int kNumInputPorts = 2;
constexpr int kInputSize[kNumInputPorts] = {1, 2};
constexpr int kContinuousStateSize = 5;
constexpr int kGeneralizedPositionSize = 2;
constexpr int kGeneralizedVelocitySize = 2;
constexpr int kMiscContinuousStateSize = 1;

constexpr double kTime = 12.0;

class LeafContextTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_.set_time(kTime);

    // Input
    context_.SetNumInputPorts(kNumInputPorts);
    for (int i = 0; i < kNumInputPorts; ++i) {
      auto port_data = std::make_unique<BasicVector<double>>(kInputSize[i]);
      auto port = std::make_unique<FreestandingInputPort>(std::move(port_data));
      context_.SetInputPort(i, std::move(port));
    }

    // Reserve a continuous state with five elements.
    context_.set_continuous_state(std::make_unique<ContinuousState<double>>(
        BasicVector<double>::Make({1.0, 2.0, 3.0, 5.0, 8.0}),
        kGeneralizedPositionSize, kGeneralizedVelocitySize,
        kMiscContinuousStateSize));

    // Reserve a difference state with two elements, of size 1 and size 2.
    std::vector<std::unique_ptr<BasicVector<double>>> xd;
    xd.push_back(BasicVector<double>::Make({128.0}));
    xd.push_back(BasicVector<double>::Make({256.0, 512.0}));
    context_.set_difference_state(
        std::make_unique<DifferenceState<double>>(std::move(xd)));

    // Reserve a modal state with one element, which is not owned.
    modal_state_ = PackValue(42);
    std::vector<AbstractValue*> xm;
    xm.push_back(modal_state_.get());
    context_.set_modal_state(std::make_unique<ModalState>(std::move(xm)));
  }

  // Mocks up a descriptor that's sufficient to read a FreestandingInputPort
  // connected to @p context at @p index.
  static const BasicVector<double>* ReadVectorInputPort(
      const Context<double>& context, int index) {
    SystemPortDescriptor<double> descriptor(
        nullptr, kInputPort, index, kVectorValued, 0, kInheritedSampling);
    return context.EvalVectorInput(nullptr, descriptor);
  }

  // Mocks up a descriptor that's sufficient to read a FreestandingInputPort
  // connected to @p context at @p index.
  static const std::string* ReadStringInputPort(
      const Context<double>& context, int index) {
    SystemPortDescriptor<double> descriptor(
        nullptr, kInputPort, index, kAbstractValued, 0, kInheritedSampling);
    return context.EvalInputValue<std::string>(nullptr, descriptor);
  }

  // Mocks up a descriptor that's sufficient to read a FreestandingInputPort
  // connected to @p context at @p index.
  static const AbstractValue* ReadAbstractInputPort(
      const Context<double>& context, int index) {
    SystemPortDescriptor<double> descriptor(
        nullptr, kInputPort, index, kAbstractValued, 0, kInheritedSampling);
    return context.EvalAbstractInput(nullptr, descriptor);
  }

  LeafContext<double> context_;
  std::unique_ptr<AbstractValue> modal_state_;
};

TEST_F(LeafContextTest, GetNumInputPorts) {
  ASSERT_EQ(kNumInputPorts, context_.get_num_input_ports());
}

TEST_F(LeafContextTest, ClearInputPorts) {
  context_.ClearInputPorts();
  EXPECT_EQ(0, context_.get_num_input_ports());
}

TEST_F(LeafContextTest, GetVectorInput) {
  LeafContext<double> context;
  context.SetNumInputPorts(2);

  // Add input port 0 to the context, but leave input port 1 uninitialized.
  std::unique_ptr<BasicVector<double>> vec(new BasicVector<double>(2));
  vec->get_mutable_value() << 5, 6;
  std::unique_ptr<FreestandingInputPort> port(
      new FreestandingInputPort(std::move(vec)));
  context.SetInputPort(0, std::move(port));

  // Test that port 0 is retrievable.
  VectorX<double> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected, ReadVectorInputPort(context, 0)->get_value());

  // Test that port 1 is nullptr.
  EXPECT_EQ(nullptr, ReadVectorInputPort(context, 1));
}

TEST_F(LeafContextTest, GetAbstractInput) {
  LeafContext<double> context;
  context.SetNumInputPorts(2);

  // Add input port 0 to the context, but leave input port 1 uninitialized.
  std::unique_ptr<AbstractValue> value(new Value<std::string>("foo"));
  std::unique_ptr<FreestandingInputPort> port(
      new FreestandingInputPort(std::move(value)));
  context.SetInputPort(0, std::move(port));

  // Test that port 0 is retrievable.
  EXPECT_EQ("foo", *ReadStringInputPort(context, 0));

  // Test that port 1 is nullptr.
  EXPECT_EQ(nullptr, ReadAbstractInputPort(context, 1));
}

// Tests that items can be stored and retrieved in the cache, even when
// the LeafContext is const.
TEST_F(LeafContextTest, SetAndGetCache) {
  const LeafContext<double>& ctx = context_;
  CacheTicket ticket = ctx.CreateCacheEntry({});
  ctx.InitCachedValue(ticket, PackValue(42));
  const AbstractValue* value = ctx.GetCachedValue(ticket);
  EXPECT_EQ(42, UnpackIntValue(value));

  ctx.SetCachedValue<int>(ticket, 43);
  EXPECT_EQ(43, UnpackIntValue(ctx.GetCachedValue(ticket)));
}

TEST_F(LeafContextTest, Clone) {
  std::unique_ptr<Context<double>> clone = context_.Clone();
  // Verify that the time was copied.
  EXPECT_EQ(kTime, clone->get_time());

  // Verify that the cloned input ports contain the same data,
  // but are different pointers.
  EXPECT_EQ(kNumInputPorts, clone->get_num_input_ports());
  for (int i = 0; i < kNumInputPorts; ++i) {
    const BasicVector<double>* context_port = ReadVectorInputPort(context_, i);
    const BasicVector<double>* clone_port = ReadVectorInputPort(*clone, i);
    EXPECT_NE(context_port, clone_port);
    EXPECT_TRUE(CompareMatrices(context_port->get_value(),
                                clone_port->get_value(), 1e-8,
                                MatrixCompareType::absolute));
  }

  // Verify that the state was copied.
  ContinuousState<double>* xc = clone->get_mutable_continuous_state();
  {
    VectorX<double> contents = xc->CopyToVector();
    VectorX<double> expected(kContinuousStateSize);
    expected << 1.0, 2.0, 3.0, 5.0, 8.0;
    EXPECT_EQ(expected, contents);
  }

  EXPECT_EQ(2, clone->get_mutable_difference_state()->size());
  BasicVector<double>* xd0 = clone->get_mutable_difference_state(0);
  BasicVector<double>* xd1 = clone->get_mutable_difference_state(1);
  {
    VectorX<double> contents = xd0->CopyToVector();
    VectorX<double> expected(1);
    expected << 128.0;
    EXPECT_EQ(expected, contents);
  }

  {
    VectorX<double> contents = xd1->CopyToVector();
    VectorX<double> expected(2);
    expected << 256.0, 512.0;
    EXPECT_EQ(expected, contents);
  }

  EXPECT_EQ(1, clone->get_mutable_modal_state()->size());
  EXPECT_EQ(42, clone->get_modal_state<int>(0));

  // Verify that the state type was preserved.
  BasicVector<double>* xc_data =
      dynamic_cast<BasicVector<double>*>(xc->get_mutable_vector());
  ASSERT_NE(nullptr, xc_data);
  EXPECT_EQ(kContinuousStateSize, xc_data->size());

  // Verify that the second-order structure was preserved.
  EXPECT_EQ(kGeneralizedPositionSize, xc->get_generalized_position().size());
  EXPECT_EQ(1.0, xc->get_generalized_position().GetAtIndex(0));
  EXPECT_EQ(2.0, xc->get_generalized_position().GetAtIndex(1));

  EXPECT_EQ(kGeneralizedVelocitySize, xc->get_generalized_velocity().size());
  EXPECT_EQ(3.0, xc->get_generalized_velocity().GetAtIndex(0));
  EXPECT_EQ(5.0, xc->get_generalized_velocity().GetAtIndex(1));

  EXPECT_EQ(kMiscContinuousStateSize, xc->get_misc_continuous_state().size());
  EXPECT_EQ(8.0, xc->get_misc_continuous_state().GetAtIndex(0));

  // Verify that changes to the cloned state do not affect the original state.
  // -- Continuous
  xc->get_mutable_generalized_velocity()->SetAtIndex(1, 42.0);
  EXPECT_EQ(42.0, xc_data->GetAtIndex(3));
  EXPECT_EQ(5.0, context_.get_continuous_state_vector().GetAtIndex(3));

  // -- Difference
  xd1->SetAtIndex(0, 1024.0);
  EXPECT_EQ(128.0, context_.get_difference_state(0)->GetAtIndex(0));

  // -- Modal (even though it's not owned in context_)
  clone->get_mutable_modal_state<int>(0) = 2048;
  EXPECT_EQ(42, context_.get_modal_state<int>(0));
}

}  // namespace systems
}  // namespace drake
