#include "drake/automotive/linear_car.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

namespace drake {
namespace automotive {
namespace {

class LinearCarTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new LinearCar<double>);
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
    input_.reset(new systems::BasicVector<double>(1));

    // Set the state to zero initially.
    systems::ContinuousState<double>* xc = continuous_state();
    EXPECT_EQ(2, xc->size());
    EXPECT_EQ(2, xc->get_misc_continuous_state().size());
    xc->SetFromVector(Eigen::VectorXd::Zero(2));
  }

  //LinearCarState<double>* continuous_state() {
  //  auto result = dynamic_cast<LinearCarState<double>*>(
  //      context_->get_mutable_continuous_state_vector());
  //  if (result == nullptr) { throw std::bad_cast(); }
  //  return result;
  //}
  systems::ContinuousState<double>* continuous_state() {
    return context_->get_mutable_continuous_state();
  }

  //void SetInputValue(double acceleration) {
  //  auto value = std::make_unique<LinearCarInput<double>>();
  //  value->set_vdot(acceleration);
  //  context_->SetInputPort(
  //      0, std::make_unique<systems::FreestandingInputPort>(std::move(value)));
  //}

  static std::unique_ptr<systems::FreestandingInputPort> MakeInput(
      std::unique_ptr<systems::BasicVector<double>> data) {
    return std::unique_ptr<systems::FreestandingInputPort>(
      new systems::FreestandingInputPort(std::move(data)));
  }

  std::unique_ptr<systems::System<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
  std::unique_ptr<systems::BasicVector<double>> input_;
};

TEST_F(LinearCarTest, Topology) {
  ASSERT_EQ(1, dut_->get_num_input_ports());
  const auto& input_descriptor = dut_->get_input_ports().at(0);
    EXPECT_EQ(systems::kVectorValued, input_descriptor.get_data_type());
  EXPECT_EQ(systems::kInputPort, input_descriptor.get_face());
  //EXPECT_EQ(LinearCarInputIndices::kNumCoordinates,
  //          input_descriptor.get_size());
  EXPECT_EQ(systems::kContinuousSampling, input_descriptor.get_sampling());

  ASSERT_EQ(1, dut_->get_num_output_ports());
  const auto& output_descriptor = dut_->get_output_ports().at(0);
  EXPECT_EQ(systems::kVectorValued, output_descriptor.get_data_type());
  EXPECT_EQ(systems::kOutputPort, output_descriptor.get_face());
  //EXPECT_EQ(LinearCarStateIndices::kNumCoordinates,
  //          output_descriptor.get_size());
  EXPECT_EQ(systems::kContinuousSampling, output_descriptor.get_sampling());
}

TEST_F(LinearCarTest, Input) {
  continuous_state()->get_mutable_vector()->SetAtIndex(0, 0.0);
  continuous_state()->get_mutable_vector()->SetAtIndex(1, 0.0);
  // Define a pointer to where the EvalOutput results end up.
  //const LinearCarState<double>* const result =
  //    dynamic_cast<const LinearCarState<double>*>(output_->get_vector_data(0));
  //ASSERT_NE(nullptr, result);
  auto result = output_->get_vector_data(0);

  // Set the input to some arbitrary value.
  //SetInputValue(6.3);
  input_->get_mutable_value() << 6.3;
  context_->SetInputPort(0, MakeInput(std::move(input_)));

  // Verify that the starting input is zero.
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(0.0, result->GetAtIndex(0));
  EXPECT_EQ(0.0, result->GetAtIndex(1));
}

TEST_F(LinearCarTest, Output) {
  // Define a pointer to where the EvalOutput results end up.
  //const LinearCarState<double>* const result =
  //    dynamic_cast<const LinearCarState<double>*>(output_->get_vector_data(0));
  //ASSERT_NE(nullptr, result);
  input_->get_mutable_value() << 1.7;
  context_->SetInputPort(0, MakeInput(std::move(input_)));

  auto result = output_->get_vector_data(0);

  // Starting state and output are all zeros.
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(0.0, result->GetAtIndex(0));
  EXPECT_EQ(0.0, result->GetAtIndex(1));

  // New state just propagates through to the output.
  continuous_state()->get_mutable_vector()->SetAtIndex(0, 1.0);
  continuous_state()->get_mutable_vector()->SetAtIndex(1, 2.0);
  dut_->EvalOutput(*context_, output_.get());
  EXPECT_EQ(1.0, result->GetAtIndex(0));
  EXPECT_EQ(2.0, result->GetAtIndex(1));
}

TEST_F(LinearCarTest, Derivatives) {

  //SetInputValue(7.4);

  // Assign to the input port a concrete value, using an
  // IntegratorTest-like implementation.
  input_->get_mutable_value() << 7.4;
  context_->SetInputPort(0, MakeInput(std::move(input_)));

  // Define a pointer to where the EvalTimeDerivatives results end up.
  //const LinearCarState<double>* const result =
  //    dynamic_cast<const LinearCarState<double>*>(
  //        derivatives_->get_mutable_vector());
  //ASSERT_NE(nullptr, result);
  auto result = derivatives_->get_mutable_vector();

  // Starting derivatives are almost all zeros, except for ego car velocity.
  dut_->EvalTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(0.0, result->GetAtIndex(0));
  EXPECT_EQ(7.4, result->GetAtIndex(1));

  // Test at a nontrivial initial condition.
  continuous_state()->get_mutable_vector()->SetAtIndex(0, 4.2);
  continuous_state()->get_mutable_vector()->SetAtIndex(1, 5.3);
  dut_->EvalTimeDerivatives(*context_, derivatives_.get());
  EXPECT_EQ(5.3, result->GetAtIndex(0));
  EXPECT_EQ(7.4, result->GetAtIndex(1));
}

}  // namespace
}  // namespace automotive
}  // namespace drake
