#include "drake/systems/primitives/zero_order_hold.h"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
#include "drake/systems/primitives/sine.h"

namespace drake {
namespace systems {
namespace {

const double kPeriod = 0.1;
const int kLength = 3;

// Define a very tight tolerance with just enough room for some roundoff.
const double kMachineTol = 10 * std::numeric_limits<double>::epsilon();

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
        TriggerType::kPeriodic);
  }
}

class ZeroOrderHoldTest : public ::testing::TestWithParam<bool> {
 protected:
  ZeroOrderHoldTest()
      : is_abstract_(GetParam()) {}
  void SetUp() override {
    DRAKE_DEMAND(kLength == 3);
    state_value_override_ << 1.0, 3.14, 2.18;
    input_value_ << 1.0, 1.0, 3.0;

    if (!is_abstract_) {
      hold_ = std::make_unique<ZeroOrderHold<double>>(kPeriod, kLength);
    } else {
      // Initialize to zero.
      hold_ = std::make_unique<ZeroOrderHold<double>>(
          kPeriod, Value<SimpleAbstractType>(Eigen::Vector3d::Zero()));
    }
    context_ = hold_->CreateDefaultContext();
    if (!is_abstract_) {
      hold_->get_input_port().FixValue(&*context_, input_value_);
    } else {
      hold_->get_input_port().FixValue(&*context_,
                                       SimpleAbstractType(input_value_));
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

  std::unique_ptr<ZeroOrderHold<double>> hold_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<CompositeEventCollection<double>> event_info_;
  const LeafCompositeEventCollection<double>* leaf_info_;

  const bool is_abstract_{};
  Eigen::Vector3d state_value_override_;
  Eigen::Vector3d input_value_;
};

// Tests that the zero-order hold has one input and one output.
TEST_P(ZeroOrderHoldTest, Topology) {
  EXPECT_EQ(1, hold_->num_input_ports());
  EXPECT_EQ(1, context_->num_input_ports());

  EXPECT_EQ(1, hold_->num_output_ports());

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
    xd.SetFromVector(output_expected);
    output = hold_->get_output_port().Eval(*context_);
  } else {
    SimpleAbstractType& state_value =
        context_->get_mutable_abstract_state<SimpleAbstractType>(0);
    state_value = SimpleAbstractType(output_expected);
    output = hold_->get_output_port().
        template Eval<SimpleAbstractType>(*context_).value();
  }
  EXPECT_EQ(output_expected, output);
}

// This test and the next one verify that the assumptions we make in ZOH about
// the behavior of Drake's rather elaborate event system are correct. If Drake's
// behavior were to change, ZOH couldn't deliver its contract so these unit
// tests provide insurance against such a change.

// Tests that when the current time is exactly on the sampling period, an update
// is requested in the future.
TEST_P(ZeroOrderHoldTest, NextUpdateTimeMustNotBeCurrentTime) {
  // Calculate the next update time *after* 0.
  context_->SetTime(0.0);
  const double t_next = hold_->CalcNextUpdateTime(*context_, event_info_.get());

  // Check that the time is correct.
  EXPECT_NEAR(kPeriod, t_next, kMachineTol);

  // Check that the action is to update.
  CheckForUpdateAction();
}

// Tests that when the current time is between updates, an update is requested
// at the appropriate time in the future.
TEST_P(ZeroOrderHoldTest, NextUpdateTimeIsInTheFuture) {
  // Calculate the next update time.
  context_->SetTime(763.2 * kPeriod);

  // Check that the time is correct.
  const double t_next = hold_->CalcNextUpdateTime(*context_, event_info_.get());
  EXPECT_NEAR(764 * kPeriod, t_next, kMachineTol);

  // Check that the action is to update.
  CheckForUpdateAction();
}

// Tests that LatchInputPortToState() updates the state.
TEST_P(ZeroOrderHoldTest, Update) {
  // Emulate an update event.
  hold_->LatchInputPortToState(&*context_);
  Eigen::Vector3d value;
  if (!is_abstract_) {
    value = hold_->get_output_port().Eval(*context_);
  } else {
    value =
        hold_->get_output_port().Eval<SimpleAbstractType>(*context_).value();
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
INSTANTIATE_TEST_SUITE_P(test, ZeroOrderHoldTest,
    ::testing::Values(false, true));

// Create a simple Diagram like this:
//    +-----------------------------------------------------+
//    |                                                     |
//    |  +------------+                  +-----------+      |
//    |  |            |                  |           |      |
//    |  |            | y=a sin(wt+p)    |           | y=x  |   y
//    |  |    sine    +------------------> u  ZOH    +---------->
//    |  |            |                  |           |      |
//    |  |            |                  |     x     |      |
//    |  +------------+                  +-----------+      |
//    |                                                     |
//    +-----------------------------------------------------+
//
// This test confirms the documented output port value is achieved both at
// initialization and after time advancement (with updates) by the Simulator.
GTEST_TEST(ZeroOrderHoldTest, UseInDiagram) {
  const double amplitude = 10.;
  const double frequency = 2. * M_PI;  // times t
  const double phase = M_PI / 4;
  const int size = 1;  // Just a 1-element output vector.

  auto sine_eval = [&](double t) {
    return amplitude * std::sin(frequency * t + phase);
  };

  DiagramBuilder<double> builder;
  auto sine =
      builder.AddSystem<Sine<double>>(amplitude, frequency, phase, size);
  sine->set_name("sine");

  auto dut = builder.AddSystem<ZeroOrderHold<double>>(
      kPeriod,  // period
      1);       // size
  dut->set_name("zoh");

  // Connect output of sine to input of zoh.
  builder.Connect(sine->get_output_port(0), dut->get_input_port());
  // Make zoh output be the output of the diagram.
  builder.ExportOutput(dut->get_output_port(), "zoh_output");
  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  const Context<double>& context = simulator.get_context();
  simulator.Initialize();

  auto eval = [&]() { return diagram->get_output_port(0).Eval(context)[0]; };

  // No update should have occurred yet.
  EXPECT_EQ(0., eval());

  simulator.AdvancePendingEvents();  // Force an update at 0.

  // Should have sampled at t=0.
  EXPECT_NEAR(sine_eval(0.), eval(), kMachineTol);

  simulator.AdvanceTo(1.5 * kPeriod);
  // Should have sampled at t=kPeriod, NOT 1.5 * kPeriod.
  EXPECT_NEAR(sine_eval(kPeriod), eval(), kMachineTol);

  simulator.AdvanceTo(9.1 * kPeriod);  // Last sample at 9 * kPeriod.
  EXPECT_NEAR(sine_eval(9 * kPeriod), eval(), kMachineTol);
}

// TODO(sherm1) There should be a unit test similar to the above for
// abstract-valued ZOH.

class SymbolicZeroOrderHoldTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const double period_sec = 0.5;
    const int size = 1;
    hold_ =
        std::make_unique<ZeroOrderHold<symbolic::Expression>>(period_sec, size);

    // Initialize the context with symbolic variables.
    context_ = hold_->CreateDefaultContext();
    hold_->get_input_port().FixValue(context_.get(),
        symbolic::Expression(symbolic::Variable("u0")));
    auto& xd = context_->get_mutable_discrete_state(0);
    xd[0] = symbolic::Variable("x0");

    update_ = hold_->AllocateDiscreteVariables();
  }

  std::unique_ptr<ZeroOrderHold<symbolic::Expression>> hold_;
  std::unique_ptr<Context<symbolic::Expression>> context_;
  std::unique_ptr<DiscreteValues<symbolic::Expression>> update_;
};

TEST_F(SymbolicZeroOrderHoldTest, Output) {
  const auto& out = hold_->get_output_port().Eval(*context_);
  EXPECT_EQ("x0", out[0].to_string());
}

TEST_F(SymbolicZeroOrderHoldTest, Update) {
  // Before latching the input, the output should just show the initial
  // state value "x0".
  EXPECT_EQ("x0", hold_->get_output_port().Eval(*context_)[0].to_string());
  hold_->LatchInputPortToState(context_.get());
  EXPECT_EQ("u0", hold_->get_output_port().Eval(*context_)[0].to_string());
}

}  // namespace
}  // namespace systems
}  // namespace drake
