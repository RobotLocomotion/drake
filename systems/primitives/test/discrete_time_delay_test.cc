#include "drake/systems/primitives/discrete_time_delay.h"

#include <cmath>
#include <limits>
#include <memory>
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
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/primitives/sine.h"

namespace drake {
namespace systems {
namespace {

// Define a very tight tolerance with just enough room for some roundoff.
const double kMachineTol = 10 * std::numeric_limits<double>::epsilon();

const double kUpdatePeriod = 0.1;
const int kBuffer = 5;  // Number of timesteps the signal is delayed.
const int kLength = 3;  // Length of vector passing through the block.

// A simple type containing a vector, to simplify checking expected values for
// a vector-valued DiscreteTimeDelay (`BasicVector`) and an abstract-valued
// DiscreteTimeDelay (`Value<SimpleAbstractType>`).
class SimpleAbstractType {
 public:
  explicit SimpleAbstractType(const Eigen::Vector3d& value)
      : value_(value) {}
  const Eigen::Vector3d& value() const { return value_; }
 private:
  Eigen::Vector3d value_;
};

Eigen::VectorXd ConcatenateAbstractBufferToVector(Context<double>* context) {
  // Preallocate a properly sized vector so it can be overwritten one buffer
  // entry at a time without memory issues.
  Eigen::VectorXd value =
      Eigen::VectorXd::Constant(kLength * (kBuffer + 1), -1);
  const int& oldest_index =
      context->template get_abstract_state<int>(kBuffer + 1);
  for (int ii = 0; ii < kBuffer + 1; ii++) {
    const SimpleAbstractType& next_value =
        context->get_abstract_state<SimpleAbstractType>((oldest_index + ii) %
                                                        (kBuffer + 1));
    value.segment(ii * kLength, kLength) = next_value.value();
  }
  return value;
}

class DiscreteTimeDelayTest : public ::testing::TestWithParam<bool> {
 protected:
  DiscreteTimeDelayTest() : is_abstract_(GetParam()) {}
  void SetUp() override {
    state_value_override_ << 1.0, 3.14, 2.18;
    input_value_ << 1.0, 1.0, 3.0;

    if (!is_abstract_) {
      delay_ = std::make_unique<DiscreteTimeDelay<double>>(
          kUpdatePeriod, kBuffer, kLength);
    } else {
      delay_ = std::make_unique<DiscreteTimeDelay<double>>(
          kUpdatePeriod, kBuffer,
          Value<SimpleAbstractType>(Eigen::Vector3d::Zero()));
    }
    context_ = delay_->CreateDefaultContext();
    if (!is_abstract_) {
      delay_->get_input_port().FixValue(context_.get(), input_value_);
    } else {
      delay_->get_input_port().FixValue(context_.get(),
                                        SimpleAbstractType(input_value_));
    }
  }

  std::unique_ptr<DiscreteTimeDelay<double>> delay_;
  std::unique_ptr<Context<double>> context_;

  const bool is_abstract_{};
  Eigen::Vector3d state_value_override_;
  Eigen::Vector3d input_value_;
};

// Tests that the time delay has one input, one output and no direct
// feedthrough.
TEST_P(DiscreteTimeDelayTest, Topology) {
  EXPECT_EQ(1, delay_->num_input_ports());
  EXPECT_EQ(1, context_->num_input_ports());

  EXPECT_EQ(1, delay_->num_output_ports());

  EXPECT_FALSE(delay_->HasAnyDirectFeedthrough());
}

// Tests that the time delay has correct type of state with the desired initial
// value.
TEST_P(DiscreteTimeDelayTest, InitialState) {
  const Eigen::VectorXd value_expected =
      Eigen::VectorXd::Zero(kLength * (kBuffer + 1));
  Eigen::VectorXd value;
  if (!is_abstract_) {
    const BasicVector<double>& xd = context_->get_discrete_state(0);
    EXPECT_EQ(kLength * (kBuffer + 1), xd.size());
    value = xd.CopyToVector();
  } else {
    value = ConcatenateAbstractBufferToVector(context_.get());
  }
  EXPECT_EQ(value_expected, value);
}

// Tests that the output is the delayed input.
TEST_P(DiscreteTimeDelayTest, Output) {
  const Eigen::Vector3d output_expected = state_value_override_;
  Eigen::Vector3d output;
  if (!is_abstract_) {
    BasicVector<double>& xd = context_->get_mutable_discrete_state(0);
    xd.get_mutable_value().head(kLength) = output_expected;
    output = delay_->get_output_port().Eval(*context_);
  } else {
    const int& oldest_index =
        context_->template get_abstract_state<int>(kBuffer + 1);
    SimpleAbstractType& next_value =
        context_->get_mutable_abstract_state<SimpleAbstractType>(oldest_index);
    next_value = SimpleAbstractType(state_value_override_);
    output = delay_->get_output_port()
                 .template Eval<SimpleAbstractType>(*context_)
                 .value();
  }
  EXPECT_EQ(output_expected, output);
}

// Tests that SaveInputToBuffer updates the state.
TEST_P(DiscreteTimeDelayTest, Update) {
  // Emulate an update event.
  delay_->SaveInputToBuffer(context_.get());
  Eigen::VectorXd value_expected =
      Eigen::VectorXd::Zero(kLength * (kBuffer + 1));
  value_expected.tail(kLength) = input_value_;
  Eigen::VectorXd value;
  // Check that the state has been updated to the input.
  if (!is_abstract_) {
    const BasicVector<double>& xd = context_->get_discrete_state(0);
    value = xd.CopyToVector();
  } else {
    value = ConcatenateAbstractBufferToVector(context_.get());
  }
  EXPECT_EQ(value_expected, value);
}

TEST_P(DiscreteTimeDelayTest, ToAutoDiff) {
  EXPECT_TRUE(is_autodiffxd_convertible(*delay_));
}

TEST_P(DiscreteTimeDelayTest, ToSymbolic) {
  EXPECT_TRUE(is_symbolic_convertible(*delay_));
}

// Instantiate parameterized test cases for is_abstract_ = {false, true}
INSTANTIATE_TEST_SUITE_P(test, DiscreteTimeDelayTest,
                        ::testing::Values(false, true));

// Create a simple Diagram to simulate:
//    +----------------------------------------------------------+
//    |                                                          |
//    |  +---------------+              +-----------+            |
//    |  |               |              |           |            |
//    |  |               | y=x        u |           | delayed_u  |
//    |  |    Counter    +-------------->   Delay   +----------------->
//    |  |               |              |           |            |
//    |  | xₖ₊₁ = xₖ + 1 |              |     x     |            |
//    |  +---------------+              +-----------+            |
//    |                                                          |
//    +----------------------------------------------------------+
// Then simulate for various intervals and make sure DiscreteTimeDelay sampled
// its discrete input at the expected times and returns the properly delayed
// signal.
GTEST_TEST(DiscreteTimeDelayTest, DiscreteSimulation) {
  DiagramBuilder<double> builder;
  // x[k+1] = x[k] + 1.  y[k] = x[k].  x[0] = 1.  (No inputs).
  auto counter = builder.AddSystem<AffineSystem<double>>(
      Vector1d::Constant(1.0),      // A.
      Eigen::MatrixXd::Zero(1, 0),  // B.
      Vector1d::Constant(1.0),      // f0.
      Vector1d::Constant(1.0),      // C.
      Eigen::MatrixXd::Zero(1, 0),  // D.
      Vector1d::Constant(0.0),      // y0.
      kUpdatePeriod / 2);
  counter->set_name("counter");

  auto delay =
      builder.AddSystem<DiscreteTimeDelay<double>>(kUpdatePeriod, kBuffer, 1);
  delay->set_name("delay");

  builder.Connect(counter->get_output_port(), delay->get_input_port());
  builder.ExportOutput(delay->get_output_port(), "delay_output");
  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  Context<double>& context = simulator.get_mutable_context();
  context.get_mutable_discrete_state(0).SetAtIndex(0, 1.0);

  auto count_eval = [&](double t) {
    return std::floor(2 * (t + kMachineTol) / kUpdatePeriod) + 1;
  };

  auto eval = [&]() { return diagram->get_output_port(0).Eval(context)[0]; };

  simulator.Initialize();
  EXPECT_EQ(0., eval());  // No update should have occurred yet.

  simulator.AdvancePendingEvents();  // Force an update at 0.
  EXPECT_EQ(0, eval());  // Should output initial value.

  // Should output initial value until delay has passed.
  simulator.AdvanceTo(0.49);
  EXPECT_EQ(0, eval());

  // Should start outputting delayed input.
  simulator.AdvanceTo(0.5);
  simulator.AdvancePendingEvents();
  EXPECT_NEAR(count_eval(0), eval(), kMachineTol);

  // Last sample at 0.9, will output value from 0.4.
  simulator.AdvanceTo(0.91);
  EXPECT_NEAR(count_eval(0.4), eval(), kMachineTol);
}

// Create a simple Diagram to simulate:
//    +-----------------------------------------------------------+
//    |                                                           |
//    |  +------------+                  +-----------+            |
//    |  |            |                  |           |            |
//    |  |            | y=a sin(wt+p)  u |           | delayed_u  |
//    |  |    sine    +------------------>   Delay   +----------------->
//    |  |            |                  |           |            |
//    |  |            |                  |     x     |            |
//    |  +------------+                  +-----------+            |
//    |                                                           |
//    +-----------------------------------------------------------+
// Then simulate for various intervals and make sure DiscreteTimeDelay sampled
// its continuous input at the expected times and returns the properly delayed
// signal.
GTEST_TEST(DiscreteTimeDelayTest, ContinuousSimulation) {
  const double sine_amplitude = 10.;
  const double sine_frequency = 2. * M_PI;
  const double sine_phase = M_PI / 4;
  const int size = 1;

  DiagramBuilder<double> builder;
  auto sine = builder.AddSystem<Sine<double>>(sine_amplitude, sine_frequency,
                                              sine_phase, size);
  sine->set_name("sine");

  auto delay = builder.AddSystem<DiscreteTimeDelay<double>>(kUpdatePeriod,
                                                            kBuffer, size);
  delay->set_name("delay");

  builder.Connect(sine->get_output_port(0), delay->get_input_port());
  builder.ExportOutput(delay->get_output_port(), "delay_output");
  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  const Context<double>& context = simulator.get_context();

  auto sine_eval = [&](double t) {
    return sine_amplitude * std::sin(sine_frequency * t + sine_phase);
  };

  auto eval = [&]() { return diagram->get_output_port(0).Eval(context)[0]; };

  simulator.Initialize();
  EXPECT_EQ(0., eval());  // No update should have occurred yet.

  simulator.AdvancePendingEvents();  // Force an update at 0.
  EXPECT_EQ(0, eval());  // Should output initial value.

  // Should output initial value until delay has passed.
  simulator.AdvanceTo(0.49);
  EXPECT_EQ(0, eval());

  // Should start outputting delayed input.
  simulator.AdvanceTo(0.5);
  simulator.AdvancePendingEvents();
  EXPECT_NEAR(sine_eval(0), eval(), kMachineTol);

  // Last sample at 0.9, will output value from 0.4.
  simulator.AdvanceTo(0.91);
  EXPECT_NEAR(sine_eval(0.4), eval(), kMachineTol);
}

class SymbolicDiscreteTimeDelayTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const int size = 1;
    delay_ = std::make_unique<DiscreteTimeDelay<symbolic::Expression>>(
        kUpdatePeriod, kBuffer, size);

    // Initialize the context with symbolic variables.
    context_ = delay_->CreateDefaultContext();
    delay_->get_input_port().FixValue(
        context_.get(), symbolic::Expression(symbolic::Variable("u0")));
    auto& xd = context_->get_mutable_discrete_state(0);
    for (int ii = 0; ii < (kBuffer + 1); ii++) {
      xd[ii] = symbolic::Variable("x" + std::to_string(ii));
    }
  }

  std::unique_ptr<DiscreteTimeDelay<symbolic::Expression>> delay_;
  std::unique_ptr<Context<symbolic::Expression>> context_;
};

// Test that the oldest (first) entry of the buffer is outputted.
TEST_F(SymbolicDiscreteTimeDelayTest, Output) {
  const auto& out = delay_->get_output_port().Eval(*context_);
  EXPECT_EQ("x0", out[0].to_string());
}

// Tests that SaveInputVectorToBuffer updates the state (i.e. slides the buffer
// forward, adding the input value to the end of the buffer).
TEST_F(SymbolicDiscreteTimeDelayTest, Update) {
  const auto& xd = context_->get_discrete_state(0);
  for (int ii = 0; ii < (kBuffer + 1); ii++) {
    EXPECT_EQ("x" + std::to_string(ii), xd[ii].to_string());
  }
  delay_->SaveInputToBuffer(context_.get());
  for (int ii = 0; ii < kBuffer; ii++) {
    EXPECT_EQ("x" + std::to_string(ii + 1), xd[ii].to_string());
  }
  EXPECT_EQ("u0", xd[kBuffer].to_string());
}

}  // namespace
}  // namespace systems
}  // namespace drake
