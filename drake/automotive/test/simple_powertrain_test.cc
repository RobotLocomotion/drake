#include "drake/automotive/simple_powertrain.h"

#include <memory>

#include "gtest/gtest.h"

using std::make_unique;

namespace drake {
namespace automotive {
namespace {

// Specify the dimension of the state vector and of each input port.
static constexpr int kStateDimension{1};
static constexpr int kInputDimension{1};

class SimplePowertrainTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new SimplePowertrain<double>(kPowertrainTimeConstant,
                                            kPowertrainGain));
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

  void SetInput(const double throttle) {
    ASSERT_NE(nullptr, dut_);
    ASSERT_NE(nullptr, context_);

    auto input_vector =
        make_unique<systems::BasicVector<double>>(1 /* throttle input */);
    input_vector->get_mutable_value() << throttle;
    context_->FixInputPort(0, std::move(input_vector));
  }

  std::unique_ptr<SimplePowertrain<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;

 private:
  const double kPowertrainTimeConstant = 0.2; /* [s] */
  const double kPowertrainGain = 1.;          /* [N] */
};

TEST_F(SimplePowertrainTest, Topology) {
  ASSERT_EQ(1, dut_->get_num_input_ports());  // Throttle input.

  const auto& input_descriptor = dut_->get_throttle_input_port();
  EXPECT_EQ(systems::kVectorValued, input_descriptor.get_data_type());
  EXPECT_EQ(kInputDimension, input_descriptor.size());

  ASSERT_EQ(1, dut_->get_num_output_ports());  // Powertrain state.

  const auto& state_descriptor = dut_->get_force_output_port();
  EXPECT_EQ(systems::kVectorValued, state_descriptor.get_data_type());
  EXPECT_EQ(kStateDimension, state_descriptor.size());

  ASSERT_EQ(false, dut_->HasAnyDirectFeedthrough());
}

TEST_F(SimplePowertrainTest, Output) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, output_);

  const double tau = dut_->get_time_constant();
  const double x{20.};

  // Obtain pointers to the state and output of the powertain model.
  auto state = context_->get_mutable_continuous_state_vector();
  const systems::BasicVector<double>* output = output_->get_vector_data(0);

  // Set the state to an arbitrary value.  No direct-feedthrough, thus the input
  // is irrelevant.
  SetInput(0.);
  (*state)[0] = x;
  dut_->CalcOutput(*context_, output_.get());

  // Verify the correctness of the output equation.
  const double expected_output = x / tau;
  EXPECT_EQ(expected_output, (*output)[0]);
}

TEST_F(SimplePowertrainTest, Derivative) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, derivatives_);

  const double gain = dut_->get_gain();
  const double tau = dut_->get_time_constant();
  const double u{4.2};
  const double x{27.};

  // Obtain pointers to the (continuous) state vector and state
  // derivative.
  auto state = context_->get_mutable_continuous_state_vector();
  const auto derivative = derivatives_->get_mutable_vector();

  // Set the throttle input to some nonzero value and the state to some
  // arbitrary value.
  SetInput(u);
  (*state)[0] = x;
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // Verify the correctness of the state derivative.
  const double expected_derivative = -x / tau + u * gain;
  EXPECT_EQ(expected_derivative, (*derivative)[0]);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
