#include "drake/systems/framework/context.h"

#include <memory>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/basic_vector.h"

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
    context_.get_mutable_time()->time_sec = kTime;
    context_.get_mutable_input()->ports.resize(kNumInputPorts);
    for (int i = 0; i < kNumInputPorts; ++i) {
      input_data_.emplace_back(new BasicVector<double>(kInputSize[i]));
      context_.get_mutable_input()->ports[i].vector_input =
          input_data_.back().get();
    }
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
  std::vector<std::unique_ptr<BasicVector<double>>> input_data_;
};

TEST_F(ContextTest, Clone) {
  std::unique_ptr<Context<double>> clone = context_.Clone();

  // Verify that the time was copied.
  EXPECT_EQ(kTime, clone->get_time().time_sec);

  // Verify that the cloned input ports point to the same data.
  EXPECT_EQ(kNumInputPorts, clone->get_input().ports.size());
  for (int i = 0; i < kNumInputPorts; ++i) {
    EXPECT_EQ(input_data_[i].get(), clone->get_input().ports[i].vector_input);
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
