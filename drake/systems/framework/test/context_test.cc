#include "drake/systems/framework/context.h"

#include <memory>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"
#include "drake/util/eigen_matrix_compare.h"

namespace drake {

using util::CompareMatrices;
using util::MatrixCompareType;

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
    // Time
    context_.get_mutable_time()->time_sec = kTime;

    // Input
    context_.SetNumInputPorts(kNumInputPorts);
    for (int i = 0; i < kNumInputPorts; ++i) {
      std::unique_ptr<VectorInterface<double>> port_data(
          new BasicVector<double>(kInputSize[i]));
      std::unique_ptr<FreestandingInputPort<double>> port(
          new FreestandingInputPort<double>(std::move(port_data)));
      context_.SetInputPort(i, std::move(port));
    }

    // State
    std::unique_ptr<BasicVector<double>> state_data(
        new BasicVector<double>(kStateSize));
    state_data->get_mutable_value() << 1.0, 2.0, 3.0, 5.0, 8.0;

    context_.get_mutable_state()->continuous_state.reset(
        new ContinuousState<double>(
            std::unique_ptr<BasicStateVector<double>>(
                new BasicStateVector<double>(std::move(state_data))),
            kGeneralizedPositionSize, kGeneralizedVelocitySize,
            kMiscContinuousStateSize));
  }

  Context<double> context_;
};

TEST_F(ContextTest, GetNumInputPorts) {
  ASSERT_EQ(kNumInputPorts, context_.get_num_input_ports());
}

TEST_F(ContextTest, ClearInputPorts) {
  context_.ClearInputPorts();
  EXPECT_EQ(0, context_.get_num_input_ports());
}

TEST_F(ContextTest, SetOutOfBoundsInputPort) {
  EXPECT_THROW(context_.SetInputPort(3, nullptr), std::out_of_range);
}

TEST_F(ContextTest, GetVectorInput) {
  Context<int> context;
  context.SetNumInputPorts(2);

  // Add input port 0 to the context, but leave input port 1 uninitialized.
  std::unique_ptr<BasicVector<int>> vec(new BasicVector<int>(2));
  vec->get_mutable_value() << 5, 6;
  std::unique_ptr<FreestandingInputPort<int>> port(
      new FreestandingInputPort<int>(std::move(vec), 0.0 /* continuous */));
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

TEST_F(ContextTest, Clone) {
  std::unique_ptr<Context<double>> clone = context_.Clone();

  // Verify that the time was copied.
  EXPECT_EQ(kTime, clone->get_time().time_sec);

  // Verify that the cloned input ports contain the same data,
  // but are different pointers.
  EXPECT_EQ(kNumInputPorts, clone->get_num_input_ports());
  for (int i = 0; i < kNumInputPorts; ++i) {
    EXPECT_NE(context_.get_vector_input(i),
              clone->get_vector_input(i));
    EXPECT_TRUE(CompareMatrices(
        context_.get_vector_input(i)->get_value(),
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
            context_.get_state().continuous_state->get_state().GetAtIndex(3));
}

}  // namespace systems
}  // namespace drake
