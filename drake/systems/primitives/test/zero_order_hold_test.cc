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
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
namespace {

const double kTenHertz = 0.1;
const int kLength = 3;

// A simple type containing a vector, to simplify checking expected values for
// a vector-valued ZOH (`BasicValue`) and an abstract-valued ZOH
// (`Value<SimpleAbstractType>`).
class SimpleAbstractType {
 public:
  explicit SimpleAbstractType(const Eigen::Vector3d& value)
      : value_(value) {}
  const Eigen::Vector3d& value() const { return value_; }
 private:
  Eigen::Vector3d value_;
};

template <typename EventListType>
void CheckForSinglePeriodicEvent(const EventListType& events) {
  EXPECT_EQ(events.size(), 1);
  // TODO(eric.cousineau): Figure out something less redundant, aside from
  // ASSERT_EQ(...), to only run the following code if the prior expectation is
  // met.
  if (events.size() == 1) {
    EXPECT_EQ(events.front()->get_trigger_type(),
        Event<double>::TriggerType::kPeriodic);
  }
}

class ZeroOrderHoldTest : public ::testing::TestWithParam<bool> {
 protected:
  ZeroOrderHoldTest()
      : is_abstract_(GetParam()) {}
  void SetUp() override {
    state_value_override_ << 1.0, 3.14, 2.18;
    input_value_ << 1.0, 1.0, 3.0;

    if (!is_abstract_) {
      hold_ = std::make_unique<ZeroOrderHold<double>>(kTenHertz, kLength);
    } else {
      // Initialize to zero.
      hold_ = std::make_unique<ZeroOrderHold<double>>(
          kTenHertz, Value<SimpleAbstractType>(Eigen::Vector3d::Zero()));
    }
    context_ = hold_->CreateDefaultContext();
    output_ = hold_->AllocateOutput(*context_);
    if (!is_abstract_) {
      context_->FixInputPort(
          0, std::make_unique<BasicVector<double>>(input_value_));
    } else {
      context_->FixInputPort(
          0, AbstractValue::Make(SimpleAbstractType(input_value_)));
    }

    event_info_ = hold_->AllocateCompositeEventCollection();
    leaf_info_ = dynamic_cast<const LeafCompositeEventCollection<double>*>(
        event_info_.get());
  }

  void CheckForUpdateAction() const {
    if (!is_abstract_) {
      CheckForSinglePeriodicEvent(
          leaf_info_->get_discrete_update_events().get_events());
    } else {
      CheckForSinglePeriodicEvent(
          leaf_info_->get_unrestricted_update_events().get_events());
    }
  }

  std::unique_ptr<System<double>> hold_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<CompositeEventCollection<double>> event_info_;
  const LeafCompositeEventCollection<double>* leaf_info_;

  const bool is_abstract_{};
  Eigen::Vector3d state_value_override_;
  Eigen::Vector3d input_value_;
};

// Tests that the zero-order hold has one input and one output.
TEST_P(ZeroOrderHoldTest, Topology) {
  EXPECT_EQ(1, hold_->get_num_input_ports());
  EXPECT_EQ(1, context_->get_num_input_ports());

  EXPECT_EQ(1, output_->get_num_ports());
  EXPECT_EQ(1, hold_->get_num_output_ports());

  EXPECT_FALSE(hold_->HasAnyDirectFeedthrough());
}

// Tests that the zero-order hold has discrete state with the desired initial
// value.
TEST_P(ZeroOrderHoldTest, InitialState) {
  const Eigen::Vector3d value_expected = Eigen::Vector3d::Zero();
  Eigen::Vector3d value;
  if (!is_abstract_) {
    const BasicVector<double>& xd = context_->get_discrete_state(0);
    EXPECT_EQ(kLength, xd.size());
    value = xd.CopyToVector();
  } else {
    const SimpleAbstractType& state_value =
        context_->get_abstract_state<SimpleAbstractType>(0);
    value = state_value.value();
  }
  EXPECT_EQ(value_expected, value);
}

// Tests that the output is the state.
TEST_P(ZeroOrderHoldTest, Output) {
  const Eigen::Vector3d output_expected = state_value_override_;
  Eigen::Vector3d output;
  if (!is_abstract_) {
    BasicVector<double>& xd = context_->get_mutable_discrete_state(0);
    xd.get_mutable_value() << output_expected;

    hold_->CalcOutput(*context_, output_.get());

    const BasicVector<double>* output_vector = output_->get_vector_data(0);
    ASSERT_NE(nullptr, output_vector);
    output = output_vector->CopyToVector();
  } else {
    SimpleAbstractType& state_value =
        context_->get_mutable_abstract_state<SimpleAbstractType>(0);
    state_value = SimpleAbstractType(output_expected);

    hold_->CalcOutput(*context_, output_.get());

    output = output_->get_data(0)->GetValue<SimpleAbstractType>().value();
  }
  EXPECT_EQ(output_expected, output);
}

// Tests that when the current time is exactly on the sampling period, a update
// is requested in the future.
TEST_P(ZeroOrderHoldTest, NextUpdateTimeMustNotBeCurrentTime) {
  // Calculate the next update time.
  context_->set_time(0.0);
  double next_t = hold_->CalcNextUpdateTime(*context_, event_info_.get());

  // Check that the time is correct.
  EXPECT_NEAR(0.1, next_t, 10e-8);

  // Check that the action is to update.
  CheckForUpdateAction();
}

// Tests that when the current time is between updates, a update is requested
// at the appropriate time in the future.
TEST_P(ZeroOrderHoldTest, NextUpdateTimeIsInTheFuture) {
  // Calculate the next update time.
  context_->set_time(76.32);

  // Check that the time is correct.
  double next_t = hold_->CalcNextUpdateTime(*context_, event_info_.get());
  EXPECT_NEAR(76.4, next_t, 10e-8);

  // Check that the action is to update.
  CheckForUpdateAction();
}

// Tests that discrete updates update the state.
TEST_P(ZeroOrderHoldTest, Update) {
  // Fire off an update event.
  Eigen::Vector3d value;
  if (!is_abstract_) {
    std::unique_ptr<DiscreteValues<double>> update =
        hold_->AllocateDiscreteVariables();
    hold_->CalcDiscreteVariableUpdates(*context_, update.get());
    // Check that the state has been updated to the input.
    const BasicVector<double>& xd = update->get_vector(0);
    value = xd.CopyToVector();
  } else {
    State<double>& state = context_->get_mutable_state();
    hold_->CalcUnrestrictedUpdate(*context_, &state);
    value = state.get_abstract_state<SimpleAbstractType>(0).value();
  }
  EXPECT_EQ(input_value_, value);
}

TEST_P(ZeroOrderHoldTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*hold_));
}

TEST_P(ZeroOrderHoldTest, ToSymbolic) {
  EXPECT_TRUE(is_symbolic_convertible(*hold_));
}

// Instantiate parameterized test cases for is_abstract_ = {false, true}
INSTANTIATE_TEST_CASE_P(test, ZeroOrderHoldTest,
    ::testing::Values(false, true));

class SymbolicZeroOrderHoldTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const double period_sec = 0.5;
    const int size = 1;
    hold_ =
        std::make_unique<ZeroOrderHold<symbolic::Expression>>(period_sec, size);

    // Initialize the context with symbolic variables.
    context_ = hold_->CreateDefaultContext();
    context_->FixInputPort(0, BasicVector<symbolic::Expression>::Make(
        symbolic::Variable("u0")));
    auto& xd = context_->get_mutable_discrete_state(0);
    xd[0] = symbolic::Variable("x0");

    output_ = hold_->AllocateOutput(*context_);
    update_ = hold_->AllocateDiscreteVariables();
  }

  std::unique_ptr<ZeroOrderHold<symbolic::Expression>> hold_;
  std::unique_ptr<Context<symbolic::Expression>> context_;
  std::unique_ptr<SystemOutput<symbolic::Expression>> output_;
  std::unique_ptr<DiscreteValues<symbolic::Expression>> update_;
};

TEST_F(SymbolicZeroOrderHoldTest, Output) {
  hold_->CalcOutput(*context_, output_.get());
  ASSERT_EQ(1, output_->get_num_ports());
  const auto& out = *output_->get_vector_data(0);
  EXPECT_EQ("x0", out[0].to_string());
}

TEST_F(SymbolicZeroOrderHoldTest, Update) {
  hold_->CalcDiscreteVariableUpdates(*context_, update_.get());
  const auto& xd = update_->get_vector(0);
  EXPECT_EQ("u0", xd[0].to_string());
}

}  // namespace
}  // namespace systems
}  // namespace drake
