#include "drake/systems/primitives/zero_order_hold.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/input_port_value.h"
#include "drake/systems/framework/output_port_value.h"

namespace drake {
namespace systems {
namespace {

const double kTenHertz = 0.1;
const int kLength = 3;

class ZeroOrderHoldTest : public ::testing::Test {
 protected:
  void SetUp() override {
    hold_ = std::make_unique<ZeroOrderHold<double>>(kTenHertz, kLength);
    context_ = hold_->CreateDefaultContext();
    output_ = hold_->AllocateOutput(*context_);
    context_->FixInputPort(0, BasicVector<double>::Make({1.0, 1.0, 3.0}));
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

  EXPECT_FALSE(hold_->HasAnyDirectFeedthrough());
}

// Tests that the zero-order hold has discrete state.
TEST_F(ZeroOrderHoldTest, ReservesState) {
  const VectorBase<double>* xd = context_->get_discrete_state(0);
  ASSERT_NE(nullptr, xd);
  EXPECT_EQ(kLength, xd->size());
}

// Tests that the output is the state.
TEST_F(ZeroOrderHoldTest, Output) {
  BasicVector<double>* xd = dynamic_cast<BasicVector<double>*>(
      context_->get_mutable_discrete_state(0));
  xd->get_mutable_value() << 1.0, 3.14, 2.18;

  hold_->CalcOutput(*context_, output_.get());

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
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction, event.action);
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
  EXPECT_EQ(DiscreteEvent<double>::kDiscreteUpdateAction, event.action);
}

// Tests that discrete updates update the state.
TEST_F(ZeroOrderHoldTest, Update) {
  // Fire off an update event.
  DiscreteEvent<double> update_event;
  update_event.action = DiscreteEvent<double>::kDiscreteUpdateAction;

  std::unique_ptr<DiscreteState<double>> update =
      hold_->AllocateDiscreteVariables();
  hold_->CalcDiscreteVariableUpdates(*context_, {update_event}, update.get());

  // Check that the state has been updated to the input.
  const VectorBase<double>* xd = update->get_discrete_state(0);
  EXPECT_EQ(1.0, xd->GetAtIndex(0));
  EXPECT_EQ(1.0, xd->GetAtIndex(1));
  EXPECT_EQ(3.0, xd->GetAtIndex(2));
}

class SymbolicZeroOrderHoldTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const double period_sec = 0.5;
    const int size = 1;
    hold_ = std::make_unique<ZeroOrderHold<symbolic::Expression>>(period_sec,
                                                                  size);

    // Initialize the context with symbolic variables.
    context_ = hold_->CreateDefaultContext();
    context_->FixInputPort(0, BasicVector<symbolic::Expression>::Make(
        symbolic::Variable("u0")));
    auto& xd = *context_->get_mutable_discrete_state(0);
    xd[0] = symbolic::Variable("x0");

    output_ = hold_->AllocateOutput(*context_);
    update_ = hold_->AllocateDiscreteVariables();
  }

  std::unique_ptr<ZeroOrderHold<symbolic::Expression>> hold_;
  std::unique_ptr<Context<symbolic::Expression>> context_;
  std::unique_ptr<SystemOutput<symbolic::Expression>> output_;
  std::unique_ptr<DiscreteState<symbolic::Expression>> update_;
};

TEST_F(SymbolicZeroOrderHoldTest, Output) {
  hold_->CalcOutput(*context_, output_.get());
  ASSERT_EQ(1, output_->get_num_ports());
  const auto& out = *output_->get_vector_data(0);
  EXPECT_EQ("x0", out[0].to_string());
}

TEST_F(SymbolicZeroOrderHoldTest, Update) {
  DiscreteEvent<symbolic::Expression> update_event;
  update_event.action =
      DiscreteEvent<symbolic::Expression>::kDiscreteUpdateAction;

  hold_->CalcDiscreteVariableUpdates(*context_, {update_event}, update_.get());
  const auto& xd = *update_->get_discrete_state(0);
  EXPECT_EQ("u0", xd[0].to_string());
}

}  // namespace
}  // namespace systems
}  // namespace drake
