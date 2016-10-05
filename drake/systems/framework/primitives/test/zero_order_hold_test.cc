#include "drake/systems/framework/primitives/zero_order_hold.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/system_output.h"

#include "gtest/gtest.h"

namespace drake {
namespace systems {
namespace {

const double kTenHertz = 0.1;
const int kLength = 3;

template <class T>
std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return std::make_unique<FreestandingInputPort>(std::move(data));
}

class ZeroOrderHoldTest : public ::testing::Test {
 protected:
  void SetUp() override {
    hold_ = std::make_unique<ZeroOrderHold<double>>(kTenHertz, kLength);
    context_ = hold_->CreateDefaultContext();
    output_ = hold_->AllocateOutput(*context_);
    context_->SetInputPort(
        0, MakeInput(BasicVector<double>::Make({1.0, 1.0, 3.0})));
  }

  std::unique_ptr<System<double>> hold_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the zero-order hold has one input and one output.
TEST_F(ZeroOrderHoldTest, Topology) {
  EXPECT_EQ(1, hold_->get_num_input_ports());
  EXPECT_EQ(1, context_->get_num_input_ports());

  EXPECT_EQ(1, output_->get_num_ports());
  EXPECT_EQ(1, hold_->get_num_output_ports());

  EXPECT_FALSE(hold_->has_any_direct_feedthrough());
}

// Tests that the zero-order hold has difference state.
TEST_F(ZeroOrderHoldTest, ReservesState) {
  const VectorBase<double>* xd = context_->get_difference_state(0);
  ASSERT_NE(nullptr, xd);
  EXPECT_EQ(kLength, xd->size());
}

// Tests that the output is the state.
TEST_F(ZeroOrderHoldTest, Output) {
  BasicVector<double>* xd = dynamic_cast<BasicVector<double>*>(
      context_->get_mutable_difference_state(0));
  xd->get_mutable_value() << 1.0, 3.14, 2.18;

  hold_->EvalOutput(*context_, output_.get());

  const BasicVector<double>* output_vector = output_->get_vector_data(0);
  ASSERT_NE(nullptr, output_vector);
  EXPECT_EQ(1.0, output_vector->GetAtIndex(0));
  EXPECT_EQ(3.14, output_vector->GetAtIndex(1));
  EXPECT_EQ(2.18, output_vector->GetAtIndex(2));
}

// Tests that when the current time is exactly on the sampling period, a update
// is requested in the future.
TEST_F(ZeroOrderHoldTest, NextUpdateTimeMustNotBeCurrentTime) {
  // Calculate the next update time.
  context_->set_time(0.0);
  UpdateActions<double> actions;
  double next_t = hold_->CalcNextUpdateTime(*context_, &actions);

  // Check that the time is correct.
  EXPECT_NEAR(0.1, next_t, 10e-8);
  EXPECT_EQ(next_t, actions.time);

  // Check that the action is to update.
  ASSERT_EQ(1u, actions.events.size());
  const DiscreteEvent<double>& event = actions.events[0];
  EXPECT_EQ(hold_.get(), event.recipient);
  EXPECT_EQ(DiscreteEvent<double>::kUpdateAction, event.action);
}

// Tests that when the current time is between updates, a update is requested
// at the appropriate time in the future.
TEST_F(ZeroOrderHoldTest, NextUpdateTimeIsInTheFuture) {
  // Calculate the next update time.
  context_->set_time(76.32);
  UpdateActions<double> actions;

  // Check that the time is correct.
  double next_t = hold_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_NEAR(76.4, next_t, 10e-8);
  EXPECT_EQ(next_t, actions.time);

  // Check that the action is to update.
  ASSERT_EQ(1u, actions.events.size());
  const DiscreteEvent<double>& event = actions.events[0];
  EXPECT_EQ(hold_.get(), event.recipient);
  EXPECT_EQ(DiscreteEvent<double>::kUpdateAction, event.action);
}

// Tests that discrete updates update the state.
TEST_F(ZeroOrderHoldTest, Update) {
  // Fire off an update event.
  DiscreteEvent<double> update_event;
  update_event.recipient = hold_.get();
  update_event.action = DiscreteEvent<double>::kUpdateAction;

  std::unique_ptr<DifferenceState<double>> update =
      hold_->AllocateDifferenceVariables();
  hold_->EvalDifferenceUpdates(*context_, {update_event}, update.get());

  // Check that the state has been updated to the input.
  const VectorBase<double>* xd = update->get_difference_state(0);
  EXPECT_EQ(1.0, xd->GetAtIndex(0));
  EXPECT_EQ(1.0, xd->GetAtIndex(1));
  EXPECT_EQ(3.0, xd->GetAtIndex(2));
}

}  // namespace
}  // namespace systems
}  // namespace drake
