#include "drake/systems/framework/primitives/integrator.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/framework/system_port_descriptor.h"

#include "gtest/gtest.h"

namespace drake {
namespace systems {
namespace {

const int kLength = 3;

class IntegratorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    integrator_.reset(new Integrator<double>(kLength));
    context_ = integrator_->CreateDefaultContext();
    derivatives_ = integrator_->AllocateTimeDerivatives();
    output_ = integrator_->AllocateOutput(*context_);
    input_.reset(new BasicVector<double>(kLength));

    // Set the state to zero initially.
    ContinuousState<double>* xc = continuous_state();
    EXPECT_EQ(3, xc->size());
    EXPECT_EQ(3, xc->get_misc_continuous_state().size());
    xc->SetFromVector(Eigen::VectorXd::Zero(kLength));
  }

  static std::unique_ptr<FreestandingInputPort> MakeInput(
      std::unique_ptr<BasicVector<double>> data) {
    return std::unique_ptr<FreestandingInputPort>(
        new FreestandingInputPort(std::move(data)));
  }

  ContinuousState<double>* continuous_state() {
    return context_->get_mutable_continuous_state();
  }

  std::unique_ptr<System<double>> integrator_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input_;
};

// Tests that the system exports the correct topology.
TEST_F(IntegratorTest, Topology) {
  ASSERT_EQ(1u, integrator_->get_input_ports().size());
  const auto& input_descriptor = integrator_->get_input_ports()[0];
  EXPECT_EQ(kVectorValued, input_descriptor.get_data_type());
  EXPECT_EQ(kInputPort, input_descriptor.get_face());
  EXPECT_EQ(kLength, input_descriptor.get_size());
  EXPECT_EQ(kContinuousSampling, input_descriptor.get_sampling());

  ASSERT_EQ(1u, integrator_->get_output_ports().size());
  const auto& output_descriptor = integrator_->get_output_ports()[0];
  EXPECT_EQ(kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(kOutputPort, output_descriptor.get_face());
  EXPECT_EQ(kLength, output_descriptor.get_size());
  EXPECT_EQ(kContinuousSampling, output_descriptor.get_sampling());
}

// Tests that the output of an integrator is its state.
TEST_F(IntegratorTest, Output) {
  ASSERT_EQ(1, context_->get_num_input_ports());
  input_->get_mutable_value() << 1.0, 2.0, 3.0;
  context_->SetInputPort(0, MakeInput(std::move(input_)));

  integrator_->EvalOutput(*context_, output_.get());

  ASSERT_EQ(1, output_->get_num_ports());
  const BasicVector<double>* output_port = output_->get_vector_data(0);
  ASSERT_NE(nullptr, output_port);

  Eigen::Vector3d expected;
  expected << 0.0, 0.0, 0.0;
  EXPECT_EQ(expected, output_port->get_value());

  continuous_state()->get_mutable_vector()->SetAtIndex(1, 42.0);
  expected << 0.0, 42.0, 0.0;
  integrator_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(expected, output_port->get_value());
}

// Tests that the derivatives of an integrator's state are its input.
TEST_F(IntegratorTest, Derivatives) {
  ASSERT_EQ(1, context_->get_num_input_ports());
  input_->get_mutable_value() << 1.0, 2.0, 3.0;
  context_->SetInputPort(0, MakeInput(std::move(input_)));

  integrator_->EvalTimeDerivatives(*context_, derivatives_.get());
  Eigen::Vector3d expected;
  expected << 1.0, 2.0, 3.0;
  EXPECT_EQ(expected, derivatives_->CopyToVector());
}

// Asserts that integrators do not have any direct feedthrough inputs.
TEST_F(IntegratorTest, IntegratorIsNotDirectFeedthrough) {
  EXPECT_FALSE(integrator_->has_any_direct_feedthrough());
}

}  // namespace
}  // namespace systems
}  // namespace drake
