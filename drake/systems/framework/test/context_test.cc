#include "drake/systems/framework/context.h"

#include <memory>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

constexpr int kNumInputPorts = 2;
constexpr int kInputSize[kNumInputPorts] = {1, 2};
constexpr int kStateSize = 5;
constexpr int kGeneralizedPositionSize = 2;
constexpr int kGeneralizedVelocitySize = 2;
constexpr int kMiscContinuousStateSize = 1;

constexpr double kTime = 12.0;

class ContextTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_.reset(new Context<double>(kNumInputPorts));
    context_->set_time(kTime);

    // Input
    for (int i = 0; i < kNumInputPorts; ++i) {
      std::unique_ptr<VectorInterface<double>> port_data(
          new BasicVector<double>(kInputSize[i]));
      std::unique_ptr<FreestandingInputPort<double>> port(
          new FreestandingInputPort<double>(std::move(port_data)));
      inputs_.push_back(port.get());
      context_->SetInputPort(i, std::move(port));
    }

    // State
    std::unique_ptr<BasicVector<double>> state_data(
        new BasicVector<double>(kStateSize));
    state_data->get_mutable_value() << 1.0, 2.0, 3.0, 5.0, 8.0;

    context_->get_mutable_state()->continuous_state.reset(
        new ContinuousState<double>(
            std::unique_ptr<BasicStateVector<double>>(
                new BasicStateVector<double>(std::move(state_data))),
            kGeneralizedPositionSize, kGeneralizedVelocitySize,
            kMiscContinuousStateSize));
  }

  std::unique_ptr<AbstractValue> PackValue(int value) {
    return std::unique_ptr<AbstractValue>(new Value<int>(value));
  }

  int UnpackValue(const AbstractValue* value) {
    return dynamic_cast<const Value<int>*>(value)->get_value();
  }

  std::vector<FreestandingInputPort<double>*> inputs_;
  std::unique_ptr<Context<double>> context_;
};

TEST_F(ContextTest, GetNumInputPorts) {
  ASSERT_EQ(kNumInputPorts, context_->get_num_input_ports());
}

TEST_F(ContextTest, SetOutOfBoundsInputPort) {
  EXPECT_THROW(context_->SetInputPort(3, nullptr), std::out_of_range);
}

TEST_F(ContextTest, GetVectorInput) {
  Context<int> context(2);

  // Add input port 0 to the context, but leave input port 1 uninitialized.
  std::unique_ptr<BasicVector<int>> vec(new BasicVector<int>(2));
  vec->get_mutable_value() << 5, 6;
  std::unique_ptr<FreestandingInputPort<int>> port(
      new FreestandingInputPort<int>(std::move(vec)));
  context.SetInputPort(0, std::move(port));

  // Test that port 0 is retrievable.
  VectorX<int> expected(2);
  expected << 5, 6;
  EXPECT_EQ(expected, context.get_vector_input(0)->get_value());

  // Test that port 1 is nullptr.
  EXPECT_EQ(nullptr, context.get_vector_input(1));

  // Test that out-of-bounds ports throw an exception.
  EXPECT_THROW(context.get_vector_input(2), std::out_of_range);
}

// Tests that, when a line of cache depends on an input port, changes to
// that input port invalidate the line of cache.
TEST_F(ContextTest, CacheDependsOnInput) {
  ContextShape<bool> dependencies = context_->MakeContextShape();
  ASSERT_EQ(kNumInputPorts, static_cast<int>(dependencies.input_ports.size()));
  dependencies.input_ports[0] = true;
  CacheTicket ticket = context_->CreateCacheEntry(dependencies, {});

  context_->SetCachedItem(ticket, PackValue(42));
  EXPECT_EQ(42, UnpackValue(context_->GetCachedItem(ticket)));

  inputs_[0]->GetMutableVectorData()->get_mutable_value()[0] = 76.0;
  EXPECT_EQ(nullptr, context_->GetCachedItem(ticket));

  // Check that changes to some other input port do not invalidate the line.
  context_->SetCachedItem(ticket, PackValue(128));
  inputs_[1]->GetMutableVectorData()->get_mutable_value()[0] = 256.0;
  EXPECT_EQ(128, UnpackValue(context_->GetCachedItem(ticket)));
}

// Tests that, when a line of cache depends on the state, changes to the
// state invalidate the line of cache.
TEST_F(ContextTest, CacheDependsOnState) {
  ContextShape<bool> dependencies = context_->MakeContextShape();
  dependencies.state.continuous_state = true;
  CacheTicket ticket = context_->CreateCacheEntry(dependencies, {});

  context_->SetCachedItem(ticket, PackValue(42));

  // Changes to an input have no effect.
  inputs_[0]->GetMutableVectorData()->get_mutable_value()[0] = 76.0;
  inputs_[1]->GetMutableVectorData()->get_mutable_value()[0] = 256.0;
  EXPECT_EQ(42, UnpackValue(context_->GetCachedItem(ticket)));

  // Changes to the state invalidate the line.
  ContinuousState<double>* xc =
      context_->get_mutable_state()->continuous_state.get();
  xc->get_mutable_state()->SetAtIndex(1, 512.0);
  EXPECT_EQ(nullptr, context_->GetCachedItem(ticket));
}

// Tests that, when a line of cache A depends on the state, and a line of cache
// B depends on A, and the state changes, both A and B are invalidated.
TEST_F(ContextTest, CacheDependsOnOtherCache) {
  ContextShape<bool> dependencies = context_->MakeContextShape();
  dependencies.state.continuous_state = true;
  CacheTicket ticket0 = context_->CreateCacheEntry(dependencies, {});
  context_->SetCachedItem(ticket0, PackValue(42));

  CacheTicket ticket1 =
      context_->CreateCacheEntry(context_->MakeContextShape(), {ticket0});
  context_->SetCachedItem(ticket1, PackValue(76));

  // Changes to the state invalidate both lines.
  ContinuousState<double>* xc =
      context_->get_mutable_state()->continuous_state.get();
  xc->get_mutable_state()->SetAtIndex(1, 512.0);
  EXPECT_EQ(nullptr, context_->GetCachedItem(ticket0));
  EXPECT_EQ(nullptr, context_->GetCachedItem(ticket1));
}

TEST_F(ContextTest, Clone) {
  std::unique_ptr<ContextBase<double>> clone = context_->Clone();

  // Verify that the time was copied.
  EXPECT_EQ(kTime, clone->get_time());

  // Verify that the cloned input ports contain the same data,
  // but are different pointers.
  EXPECT_EQ(kNumInputPorts, clone->get_num_input_ports());
  for (int i = 0; i < kNumInputPorts; ++i) {
    EXPECT_NE(context_->get_vector_input(i), clone->get_vector_input(i));
    EXPECT_TRUE(CompareMatrices(context_->get_vector_input(i)->get_value(),
                                clone->get_vector_input(i)->get_value(), 1e-8,
                                MatrixCompareType::absolute));
  }

  // Verify that the state was copied.
  ContinuousState<double>* xc =
      clone->get_mutable_state()->continuous_state.get();
  VectorX<double> contents = xc->get_state().CopyToVector();
  VectorX<double> expected(kStateSize);
  expected << 1.0, 2.0, 3.0, 5.0, 8.0;
  EXPECT_EQ(expected, contents);

  // Verify that the state type was preserved.
  BasicStateVector<double>* xc_data =
      dynamic_cast<BasicStateVector<double>*>(xc->get_mutable_state());
  ASSERT_NE(nullptr, xc_data);
  EXPECT_EQ(kStateSize, xc_data->size());

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
  xc->get_mutable_generalized_velocity()->SetAtIndex(1, 42.0);
  EXPECT_EQ(42.0, xc_data->GetAtIndex(3));
  EXPECT_EQ(5.0,
            context_->get_state().continuous_state->get_state().GetAtIndex(3));
}

}  // namespace systems
}  // namespace drake
