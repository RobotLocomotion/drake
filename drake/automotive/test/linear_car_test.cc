#include "drake/automotive/linear_car.h"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

namespace drake {
namespace automotive {
namespace {

class LinearCarTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Initialize LinearCar with x, v = 0.
    dut_.reset(new LinearCar<double>(0.0, 0.0));
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
    input_.reset(new systems::BasicVector<double>(1));

    // Set the state to zero initially.
    systems::ContinuousState<double>* xc = continuous_state();
    EXPECT_EQ(2, xc->size());
    EXPECT_EQ(1, xc->get_generalized_position().size());
    EXPECT_EQ(1, xc->get_generalized_velocity().size());
    EXPECT_EQ(0, xc->get_misc_continuous_state().size());
    xc->SetFromVector(Eigen::VectorXd::Zero(2));
  }

  systems::ContinuousState<double>* continuous_state() {
    return context_->get_mutable_continuous_state();
  }

  std::unique_ptr<systems::System<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
  std::unique_ptr<systems::BasicVector<double>> input_;
};

TEST_F(LinearCarTest, Topology) {
  ASSERT_EQ(1, dut_->get_num_input_ports());
  const auto& input_descriptor = dut_->get_input_port(0);
  EXPECT_EQ(systems::kVectorValued, input_descriptor.get_data_type());
  EXPECT_EQ(1 /* one input: vdot */, input_descriptor.size());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& output_descriptor = dut_->get_output_port(0);
  EXPECT_EQ(systems::kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(2 /* two outputs: x, v */, output_descriptor.size());
}

TEST_F(LinearCarTest, Output) {
  input_->get_mutable_value() << 1.7;
  context_->FixInputPort(0, std::move(input_));

  // Define a pointer to where the CalcOutput results end up.
  auto result = output_->get_vector_data(0);

  // Expect the state and output vectors are all zeros initially.
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(0.0, result->GetAtIndex(0));
  EXPECT_EQ(0.0, result->GetAtIndex(1));

  // New state propagates to the output.
  (*continuous_state()->get_mutable_generalized_position())[0] = 1.0;
  (*continuous_state()->get_mutable_generalized_velocity())[0] = 2.0;
  dut_->CalcOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->GetAtIndex(0));
  EXPECT_EQ(2.0, result->GetAtIndex(1));
}

TEST_F(LinearCarTest, Derivatives) {
  // Assign an arbitrary input value.
  input_->get_mutable_value() << 7.4;
  context_->FixInputPort(0, std::move(input_));

  // Define a pointer to where the EvalTimeDerivatives results end up.
  auto result = derivatives_->get_mutable_vector();

  // Starting derivatives are almost all zeros, except for ego car velocity.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->GetAtIndex(0));
  EXPECT_EQ(7.4, result->GetAtIndex(1));

  // Test at a nontrivial initial condition.
  (*continuous_state()->get_mutable_generalized_position())[0] = 4.2;
  (*continuous_state()->get_mutable_generalized_velocity())[0] = 5.3;
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(5.3, result->GetAtIndex(0));
  EXPECT_EQ(7.4, result->GetAtIndex(1));
}

}  // namespace
}  // namespace automotive
}  // namespace drake
