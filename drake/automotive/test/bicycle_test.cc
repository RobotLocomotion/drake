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
    force_input_.reset(new systems::BasicVector<double>(
        force_input_dimension_));

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

  void SetInputs(const double& steering_input, const double& force_input) {
    DRAKE_DEMAND(steering_input_ != nullptr);
    DRAKE_DEMAND(force_input_ != nullptr);
    DRAKE_DEMAND(dut_ != nullptr);
    DRAKE_DEMAND(context_ != nullptr);

    steering_input_->SetAtIndex(0, steering_input);
    force_input_->SetAtIndex(0, force_input);

    const int kSteeringIndex = dut_->get_steering_input_port().get_index();
    const int kForceIndex = dut_->get_force_input_port().get_index();
    context_->FixInputPort(kSteeringIndex, std::move(steering_input_));
    context_->FixInputPort(kForceIndex, std::move(force_input_));
  }

  const int state_dimension_{6};
  const int steering_input_dimension_{1};
  const int force_input_dimension_{1};

  std::unique_ptr<Bicycle<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
  std::unique_ptr<systems::BasicVector<double>> steering_input_;
  std::unique_ptr<systems::BasicVector<double>> force_input_;
};

TEST_F(BicycleTest, Topology) {
  ASSERT_EQ(2, dut_->get_num_input_ports()); /* steering angle, force input */

  const int kSteeringIndex = dut_->get_steering_input_port().get_index();
  const auto& steering_input_descriptor = dut_->get_input_port(kSteeringIndex);
  EXPECT_EQ(systems::kVectorValued, steering_input_descriptor.get_data_type());
  EXPECT_EQ(steering_input_dimension_, steering_input_descriptor.size());

  const int kForceIndex = dut_->get_force_input_port().get_index();
  const auto& force_input_descriptor = dut_->get_input_port(kForceIndex);
  EXPECT_EQ(systems::kVectorValued, force_input_descriptor.get_data_type());
  EXPECT_EQ(force_input_dimension_, force_input_descriptor.size());

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

  systems::ContinuousState<double>* xc = continuous_state();
  xc->SetFromVector(Vector6<double>::Ones());

  dut_->CalcOutput(*context_, output_.get());

  const Vector6<double> result = output->CopyToVector();
  EXPECT_TRUE(CompareMatrices(result, Vector6<double>::Ones(), kTolerance,
                              MatrixCompareType::absolute));
}

// Tests the consistency of the derivatives when a trivial set of states is
// provided, with no steering and some positive input force.
TEST_F(BicycleTest, TrivialDerivatives) {
  const double kForceInput = 10.;
  const double kVelocityState = 1.;

  // Keep the steering angle zero and set the force to some positive value.
  SetInputs(0., kForceInput);

  // Set all the states to zero except velocity.
  systems::ContinuousState<double>* xc = continuous_state();
  xc->SetFromVector(Vector6<double>::Zero());
  xc->get_mutable_vector()->SetAtIndex(3, kVelocityState);  // Keep the velocity
                                                            // positive.

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  const Vector6<double> result = derivatives_->CopyToVector();

  // We expect all the derivatives to be identically zero except the v_dot and
  // sx_dot; require that v_dot is positive for positive force input.
  EXPECT_EQ(0., result(0));
  EXPECT_EQ(0., result(1));
  EXPECT_EQ(0., result(2));
  EXPECT_LT(0., result(3));
  EXPECT_EQ(kVelocityState, result(4));
  EXPECT_EQ(0., result(5));
}

// Tests that one equation has terms that cancel for some parameter-independent
// settings, and that the remaining equations are still consistent, including
// the kinematics equations.
TEST_F(BicycleTest, DerivativesPositiveSlipAnglePositiveSteeringAngle) {
  const double kSteeringInput = 1.;  // An angle in the first-quadrant.
  const double kForceInput = 10.;
  const double kVelocityState = 1.;

  // Set the steering angle and force to some positive values.
  SetInputs(kSteeringInput, kForceInput);

  // Set β to δ / 2 and the velocity to some positive value.
  systems::ContinuousState<double>* xc = continuous_state();
  xc->SetFromVector(Vector6<double>::Zero());
  xc->get_mutable_vector()->SetAtIndex(2, kSteeringInput / 2.);
  xc->get_mutable_vector()->SetAtIndex(3, kVelocityState);

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  const Vector6<double> result = derivatives_->CopyToVector();

  // We expect the first and third terms in β_dot to cancel, and Ψ_ddot,
  // v_dot, sx_dot, and sy_dot to return strictly-positive values.
  EXPECT_LT(0., result(1));
  EXPECT_EQ(0., result(2));
  EXPECT_LT(0., result(3));
  EXPECT_LT(0., result(4));
  EXPECT_LT(0., result(5));
}

// Tests the consistency of the derivatives and kinematic relationships upon
// feeding in a negative slip angle, keeping the other parameters the same as
// the previous case.
TEST_F(BicycleTest, DerivativesNegativeSlipAnglePositiveSteeringAngle) {
  const double kSteeringInput = 1.;  // An angle in the fourth-quadrant.
  const double kForceInput = 10.;
  const double kVelocityState = 1.;

  // Set the steering angle and force to some positive values.
  SetInputs(kSteeringInput, kForceInput);

  // Set β to -δ and the velocity to some positive value.
  systems::ContinuousState<double>* xc = continuous_state();
  xc->SetFromVector(Vector6<double>::Zero());
  xc->get_mutable_vector()->SetAtIndex(2, -kSteeringInput);
  xc->get_mutable_vector()->SetAtIndex(3, kVelocityState);

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  const Vector6<double> result = derivatives_->CopyToVector();

  // We expect β_dot and v_dot to return strictly-positive values. sx_dot and
  // sy_dot each, respectively positive and negative values.  Note that Ψ_ddot
  // is indeterminate.
  EXPECT_LT(0., result(2));
  EXPECT_LT(0., result(3));
  EXPECT_LT(0., result(4));
  EXPECT_GT(0., result(5));
}

// Tests the consistency of the derivatives and kinematic relationships upon
// assigning a positive angular yaw rate.
TEST_F(BicycleTest, DerivativesPositiveYawRate) {
  const double kYawRateState = 2.;
  const double kVelocityState = 1e3;  // Unrealistic velocity that reasonably
                                      // enforces the condition that
                                      // abs(Cr * lr - Cf * lf) < 1e6 * mass.

  // Keep the steering angle and force zero.
  SetInputs(0., 0.);

  // Set Ψ_dot and the velocity to some positive values.
  systems::ContinuousState<double>* xc = continuous_state();
  xc->SetFromVector(Vector6<double>::Zero());
  xc->get_mutable_vector()->SetAtIndex(1, kYawRateState);
  xc->get_mutable_vector()->SetAtIndex(3, kVelocityState);

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  const Vector6<double> result = derivatives_->CopyToVector();

  // We expect β_dot to be negative and Ψ_ddot to be positive.
  EXPECT_GT(0., result(1));
  EXPECT_GT(0., result(2));
}

}  // namespace
}  // namespace automotive
}  // namespace drake
