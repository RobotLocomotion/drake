#include "drake/automotive/bicycle.h"

#include <cmath>
#include <memory>
#include <utility>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace automotive {
namespace {

class BicycleTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new Bicycle<double>());

    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
    steering_input_.reset(new systems::BasicVector<double>(
        steering_input_dimension_));
    pt_input_.reset(new systems::BasicVector<double>(
        powertrain_input_dimension_));

    // Set the state to zero initially.
    systems::ContinuousState<double>* xc = continuous_state();
    EXPECT_EQ(state_dimension_, xc->size());
    EXPECT_EQ(1, xc->get_generalized_position().size());  // Expect Psi.
    EXPECT_EQ(1, xc->get_generalized_velocity().size());  // Expect Psi_dot.
    EXPECT_EQ(state_dimension_ - 2, xc->get_misc_continuous_state().size());
    xc->SetFromVector(Eigen::VectorXd::Zero(state_dimension_));
  }

  systems::ContinuousState<double>* continuous_state() {
    return context_->get_mutable_continuous_state();
  }

  void SetInputs(const double& steering_input, const double& pt_input) {
    DRAKE_DEMAND(steering_input_ != nullptr);
    DRAKE_DEMAND(pt_input_ != nullptr);
    DRAKE_DEMAND(dut_ != nullptr);
    DRAKE_DEMAND(context_ != nullptr);

    steering_input_->SetAtIndex(0, steering_input);
    pt_input_->SetAtIndex(0, pt_input);

    const int kSteeringIndex = dut_->get_steering_input_port().get_index();
    const int kPowertrainIndex = dut_->get_pt_input_port().get_index();
    context_->FixInputPort(kSteeringIndex, std::move(steering_input_));
    context_->FixInputPort(kPowertrainIndex, std::move(pt_input_));
  }

  const int state_dimension_{6};
  const int steering_input_dimension_{1};
  const int powertrain_input_dimension_{1};

  std::unique_ptr<Bicycle<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
  std::unique_ptr<systems::BasicVector<double>> steering_input_;
  std::unique_ptr<systems::BasicVector<double>> pt_input_;
};

TEST_F(BicycleTest, Topology) {
  ASSERT_EQ(2, dut_->get_num_input_ports()); /* steering angle, force input */

  const int kSteeringIndex = dut_->get_steering_input_port().get_index();
  const auto& steering_input_descriptor = dut_->get_input_port(kSteeringIndex);
  EXPECT_EQ(systems::kVectorValued, steering_input_descriptor.get_data_type());
  EXPECT_EQ(steering_input_dimension_, steering_input_descriptor.size());

  const int kPowertrainIndex = dut_->get_pt_input_port().get_index();
  const auto& pt_input_descriptor = dut_->get_input_port(kPowertrainIndex);
  EXPECT_EQ(systems::kVectorValued, pt_input_descriptor.get_data_type());
  EXPECT_EQ(powertrain_input_dimension_, pt_input_descriptor.size());

  ASSERT_EQ(1, dut_->get_num_output_ports()); /* state vector */

  const auto& state_descriptor = dut_->get_output_port(0);
  EXPECT_EQ(systems::kVectorValued, state_descriptor.get_data_type());
  EXPECT_EQ(state_dimension_, state_descriptor.size());
}

TEST_F(BicycleTest, Output) {
  const double kTolerance = 1e-10;

  // Keep the steering angle zero and the throttle to some positive value.
  SetInputs(0., 10.);

  auto output = output_->get_vector_data(0);

  systems::ContinuousState<double>* xc =
      context_->get_mutable_continuous_state();
  xc->SetFromVector(Vector6<double>::Ones());

  dut_->CalcOutput(*context_, output_.get());

  const Vector6<double> result = output->CopyToVector();
  EXPECT_TRUE(CompareMatrices(result, Vector6<double>::Ones(), kTolerance,
                              MatrixCompareType::absolute));
}

TEST_F(BicycleTest, Derivatives) {
  const double kTolerance = 1e-10;

  // Keep the steering angle zero and the throttle to some positive value.
  SetInputs(0., 0.5);

  // Set all the states to one.
  systems::ContinuousState<double>* xc =
      context_->get_mutable_continuous_state();
  xc->SetFromVector(Vector6<double>::Ones());

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  const Vector6<double> result = derivatives_->CopyToVector();
  Vector6<double> expected_result;
  expected_result << -0.3012, 1.3818, 1., 3.1355, 0.7734, -8.7835;

  EXPECT_TRUE(CompareMatrices(result, expected_result, kTolerance,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace automotive
}  // namespace drake
