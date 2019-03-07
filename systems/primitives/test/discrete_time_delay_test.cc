#include "drake/systems/primitives/discrete_time_delay.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/signal_logger.h"

namespace drake {
namespace systems {
namespace {

const double kTenHertz = 0.1;
const double kBuffer = 5;
const int kLength = 3;

template <typename EventListType>
void CheckForSinglePeriodicEvent(const EventListType& events) {
  EXPECT_EQ(events.size(), 1);
  if (events.size() == 1) {
    EXPECT_EQ(events.front()->get_trigger_type(), TriggerType::kPeriodic);
  }
}

class DiscreteTimeDelayTest : public ::testing::TestWithParam<bool> {
 protected:
  DiscreteTimeDelayTest() : is_abstract_(GetParam()) {}
  void SetUp() override {
    state_value_override_ << 1.0, 3.14, 2.18;
    input_value_ << 1.0, 1.0, 3.0;

    hold_ = std::make_unique<DiscreteTimeDelay<double>>(
        kTenHertz, kBuffer, kLength);
    context_ = hold_->CreateDefaultContext();
    context_->FixInputPort(
        0, std::make_unique<BasicVector<double>>(input_value_));

    event_info_ = hold_->AllocateCompositeEventCollection();
    leaf_info_ = dynamic_cast<const LeafCompositeEventCollection<double>*>(
        event_info_.get());
  }

  void CheckForUpdateAction() const {
    CheckForSinglePeriodicEvent(
        leaf_info_->get_discrete_update_events().get_events());
  }

  std::unique_ptr<DiscreteTimeDelay<double>> hold_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<CompositeEventCollection<double>> event_info_;
  const LeafCompositeEventCollection<double>* leaf_info_;

  const bool is_abstract_{};
  Eigen::Vector3d state_value_override_;
  Eigen::Vector3d input_value_;
};

// Tests that the time delay has one input, one output and no direct
// feedthrough.
TEST_P(DiscreteTimeDelayTest, Topology) {
  EXPECT_EQ(1, hold_->get_num_input_ports());
  EXPECT_EQ(1, context_->get_num_input_ports());

  EXPECT_EQ(1, hold_->get_num_output_ports());

  EXPECT_FALSE(hold_->HasAnyDirectFeedthrough());
}

// Tests that the time delay has correct type of state with the desired initial
// value.
TEST_P(DiscreteTimeDelayTest, InitialState) {
  const Eigen::VectorXd value_expected =
      Eigen::VectorXd::Zero(kLength * kBuffer);
  Eigen::VectorXd value;
  const BasicVector<double>& xd = context_->get_discrete_state(0);
  EXPECT_EQ(kLength * kBuffer, xd.size());
  value = xd.CopyToVector();
  EXPECT_EQ(value_expected, value);
}

// Tests that the output is the delayed input (head of the state vector.
TEST_P(DiscreteTimeDelayTest, Output) {
  const Eigen::Vector3d output_expected = state_value_override_;
  Eigen::Vector3d output;
  BasicVector<double>& xd = context_->get_mutable_discrete_state(0);
  xd.get_mutable_value().head(kLength) << output_expected;
  output = hold_->get_output_port().Eval(*context_);
  EXPECT_EQ(output_expected, output);
}

// Tests that when the current time is exactly on the sampling period, a update
// is requested in the future.
TEST_P(DiscreteTimeDelayTest, NextUpdateTimeMustNotBeCurrentTime) {
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
TEST_P(DiscreteTimeDelayTest, NextUpdateTimeIsInTheFuture) {
  // Calculate the next update time.
  context_->set_time(76.32);

  // Check that the time is correct.
  double next_t = hold_->CalcNextUpdateTime(*context_, event_info_.get());
  EXPECT_NEAR(76.4, next_t, 10e-8);

  // Check that the action is to update.
  CheckForUpdateAction();
}

// Tests that SaveInputVectorToBuffer updates the state.
TEST_P(DiscreteTimeDelayTest, Update) {
  // Emulate an update event.
  hold_->SaveInputToBuffer(&*context_);
  Eigen::VectorXd value_expected = Eigen::VectorXd::Zero(kLength * kBuffer);
  value_expected.tail(kLength) = input_value_;
  Eigen::VectorXd value;
  // Check that the state has been updated to the input.
  const BasicVector<double>& xd = context_->get_discrete_state(0);
  value = xd.CopyToVector();
  EXPECT_EQ(value_expected, value);
}

// Tests that the this block delays the input signal during simulation
TEST_P(DiscreteTimeDelayTest, Simulation) {
  DiagramBuilder<double> builder;
  auto delay = builder.AddSystem(std::move(hold_));

  // x[k+1] = 2*x[k].  y[k] = x[k].  (No inputs).
  auto source = builder.AddSystem<LinearSystem<double>>(
      Vector1d::Constant(2.0),               // A.
      Eigen::MatrixXd::Zero(1, 0),           // B.
      Eigen::MatrixXd::Constant(3, 1, 1.0),  // C.
      Eigen::MatrixXd::Zero(3, 0),           // D.
      kTenHertz);
  source->set_name("source");

  builder.Cascade(*source, *delay);
  auto logger = LogOutput(delay->get_output_port(), &builder);
  auto logger2 = LogOutput(source->get_output_port(), &builder);

  auto diagram = builder.Build();

  // Simulate the simple system from x(0) = 1.0.
  Simulator<double> simulator(*diagram);
  Context<double>& context = simulator.get_mutable_context();
  context.get_mutable_discrete_state(1).SetAtIndex(0, 1.0);

  simulator.Initialize();
  // The Simulator schedules SignalLogger's default per-step event, which
  // performs data logging.
  simulator.StepTo(1);

  const auto& delayed_data = logger->data();
  const auto& raw_data = logger2->data();
  Eigen::MatrixXd expected_data(3, 11);
  expected_data << Eigen::MatrixXd::Zero(3, 5), raw_data.leftCols(6);
  EXPECT_TRUE(CompareMatrices(expected_data, delayed_data));
}

TEST_P(DiscreteTimeDelayTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*hold_));
}

TEST_P(DiscreteTimeDelayTest, ToSymbolic) {
  EXPECT_TRUE(is_symbolic_convertible(*hold_));
}

// Instantiate parameterized test cases for is_abstract_ = {false, true}
INSTANTIATE_TEST_CASE_P(test, DiscreteTimeDelayTest, ::testing::Values(false));

class SymbolicDiscreteTimeDelayTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const int size = 1;
    hold_ = std::make_unique<DiscreteTimeDelay<symbolic::Expression>>(
        kTenHertz, kBuffer, size);

    // Initialize the context with symbolic variables.
    context_ = hold_->CreateDefaultContext();
    context_->FixInputPort(
        0, BasicVector<symbolic::Expression>::Make(symbolic::Variable("u0")));
    auto& xd = context_->get_mutable_discrete_state(0);
    for (int ii = 0; ii < kBuffer; ii++) {
      xd[ii] = symbolic::Variable("x0");
    }
  }

  std::unique_ptr<DiscreteTimeDelay<symbolic::Expression>> hold_;
  std::unique_ptr<Context<symbolic::Expression>> context_;
};

TEST_F(SymbolicDiscreteTimeDelayTest, Output) {
  const auto& out = hold_->get_output_port().Eval(*context_);
  EXPECT_EQ("x0", out[0].to_string());
}

TEST_F(SymbolicDiscreteTimeDelayTest, Update) {
  hold_->SaveInputToBuffer(&*context_);
  const auto& xd = context_->get_discrete_state(0);
  for (int ii = 0; ii < kBuffer - 1; ii++) {
    EXPECT_EQ("x0", xd[ii].to_string());
  }
  EXPECT_EQ("u0", xd[kBuffer - 1].to_string());
}

}  // namespace
}  // namespace systems
}  // namespace drake
