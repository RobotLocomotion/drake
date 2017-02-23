#include "drake/automotive/bicycle.h"

#include <cmath>
#include <memory>
#include <utility>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace automotive {
namespace {

static constexpr int kStateDimension{6};
static constexpr int kSteeringInputDimension{1};
static constexpr int kForceInputDimension{1};

class BicycleTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new Bicycle<double>());

    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
    steering_input_.reset(
        new systems::BasicVector<double>(kSteeringInputDimension));
    force_input_.reset(new systems::BasicVector<double>(kForceInputDimension));

    // Set the state to zero initially.
    systems::ContinuousState<double>* xc = continuous_state();
    EXPECT_EQ(kStateDimension, xc->size());
    EXPECT_EQ(1, xc->get_generalized_position().size()); /* q = Psi */
    EXPECT_EQ(1, xc->get_generalized_velocity().size()); /* v = Psi_dot */
    EXPECT_EQ(kStateDimension - 2, xc->get_misc_continuous_state().size());
    xc->SetFromVector(Eigen::VectorXd::Zero(kStateDimension));
  }

  systems::ContinuousState<double>* continuous_state() {
    return context_->get_mutable_continuous_state();
  }

  void SetInputs(const double& steering_input, const double& force_input) {
    DRAKE_DEMAND(steering_input_ != nullptr);
    DRAKE_DEMAND(force_input_ != nullptr);
    DRAKE_DEMAND(dut_ != nullptr);
    DRAKE_DEMAND(context_ != nullptr);

    (*steering_input_)[0] = steering_input;
    (*force_input_)[0] = force_input;

    const int kSteeringIndex = dut_->get_steering_input_port().get_index();
    const int kForceIndex = dut_->get_force_input_port().get_index();
    context_->FixInputPort(kSteeringIndex, std::move(steering_input_));
    context_->FixInputPort(kForceIndex, std::move(force_input_));
  }

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
  EXPECT_EQ(kSteeringInputDimension, steering_input_descriptor.size());

  const int kForceIndex = dut_->get_force_input_port().get_index();
  const auto& force_input_descriptor = dut_->get_input_port(kForceIndex);
  EXPECT_EQ(systems::kVectorValued, force_input_descriptor.get_data_type());
  EXPECT_EQ(kForceInputDimension, force_input_descriptor.size());

  ASSERT_EQ(1, dut_->get_num_output_ports()); /* state vector */

  const auto& state_descriptor = dut_->get_output_port(0);
  EXPECT_EQ(systems::kVectorValued, state_descriptor.get_data_type());
  EXPECT_EQ(kStateDimension, state_descriptor.size());
}

TEST_F(BicycleTest, Output) {
  const double kTolerance = 1e-10;

  // Keep the steering angle and the applied force zero.
  SetInputs(1., 10.);

  auto output = output_->get_vector_data(0);

  systems::ContinuousState<double>* xc = continuous_state();
  xc->SetFromVector(Vector6<double>::Ones());  // Set all the states to one.

  dut_->CalcOutput(*context_, output_.get());
  const Vector6<double> result = output->CopyToVector();

  // Expect that the output matches the states, since there is no feedthrough.
  EXPECT_TRUE(CompareMatrices(result, Vector6<double>::Ones(), kTolerance,
                              MatrixCompareType::absolute));
}

// Tests the consistency of the derivatives when a trivial set of states is
// provided, with no steering and some positive input force.
TEST_F(BicycleTest, TrivialDerivatives) {
  const double kForceInput = 10.;
  const double kVelocityState = 1.;

  // Keep the steering angle zero and set the force to a positive value.
  SetInputs(0., kForceInput);

  // Set all the states to zero except velocity, which must be kept positive.
  systems::ContinuousState<double>* xc = continuous_state();
  xc->SetFromVector(Vector6<double>::Zero());
  (*xc->get_mutable_vector())[3] = kVelocityState; /* v */

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  const Vector6<double> result = derivatives_->CopyToVector();

  // We expect all the derivatives to be zero except the v_dot and sx_dot, as we
  // apply a positive force input and zero steering angle translates into
  // along-track motion.
  EXPECT_EQ(0., result(0));             /* Ψ_dot */
  EXPECT_EQ(0., result(1));             /* Ψ_ddot */
  EXPECT_EQ(0., result(2));             /* β_dot */
  EXPECT_LT(0., result(3));             /* v_dot */
  EXPECT_EQ(kVelocityState, result(4)); /* sx_dot */
  EXPECT_EQ(0., result(5));             /* sy_dot */
}

// Tests that one equation has terms that cancel for some parameter-independent
// settings, and that the remaining equations are still consistent, including
// the kinematics equations.
TEST_F(BicycleTest, DerivativesPositiveSlipAnglePositiveSteeringAngle) {
  const double kSteeringInput = 1.;  // An angle in the first quadrant.
  const double kForceInput = 10.;
  const double kVelocityState = 1.;

  // Set the steering angle and force to positive values.
  SetInputs(kSteeringInput, kForceInput);

  // Set β to δ / 2 and the velocity to a positive value.
  systems::ContinuousState<double>* xc = continuous_state();
  xc->SetFromVector(Vector6<double>::Zero());
  (*xc->get_mutable_vector())[2] = kSteeringInput / 2.; /* β */
  (*xc->get_mutable_vector())[3] = kVelocityState;      /* v */

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  const Vector6<double> result = derivatives_->CopyToVector();

  // We expect the first and third terms in β_dot to cancel, and Ψ_ddot, v_dot
  // to be strictly positive. We expect sx_dot, and sy_dot to be
  // strictly-positive as the steering angle is in the first quadrant.
  EXPECT_LT(0., result(1)); /* Ψ_ddot */
  EXPECT_EQ(0., result(2)); /* β_dot */
  EXPECT_LT(0., result(3)); /* v_dot */
  EXPECT_LT(0., result(4)); /* sx_dot */
  EXPECT_LT(0., result(5)); /* sy_dot */
}

// Tests the consistency of the derivatives and kinematic relationships upon
// feeding in a negative slip angle, keeping the other parameters the same as
// the previous case.
TEST_F(BicycleTest, DerivativesNegativeSlipAnglePositiveSteeringAngle) {
  const double kSteeringInput = 1.;  // An angle in the fourth quadrant.
  const double kForceInput = 10.;
  const double kVelocityState = 1.;

  // Set the steering angle and force to positive values.
  SetInputs(kSteeringInput, kForceInput);

  // Set β to -δ and the velocity to a positive value.
  systems::ContinuousState<double>* xc = continuous_state();
  xc->SetFromVector(Vector6<double>::Zero());
  (*xc->get_mutable_vector())[2] = -kSteeringInput; /* β */
  (*xc->get_mutable_vector())[3] = kVelocityState;  /* v */

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  const Vector6<double> result = derivatives_->CopyToVector();

  // We expect β_dot and v_dot to return strictly-positive values. sx_dot and
  // sy_dot, respectively, return positive and negative values, as the steering
  // angle is in the fourth quadrant.  Note that Ψ_ddot is indeterminate.
  EXPECT_LT(0., result(2)); /* β_dot */
  EXPECT_LT(0., result(3)); /* v_dot */
  EXPECT_LT(0., result(4)); /* sx_dot */
  EXPECT_GT(0., result(5)); /* sy_dot */
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

  // Set Ψ_dot and the velocity to positive values.
  systems::ContinuousState<double>* xc = continuous_state();
  xc->SetFromVector(Vector6<double>::Zero());
  (*xc->get_mutable_vector())[1] = kYawRateState;  /* Ψ_dot */
  (*xc->get_mutable_vector())[3] = kVelocityState; /* β */

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  const Vector6<double> result = derivatives_->CopyToVector();

  // We expect both β_dot and Ψ_ddot to be strictly negative.
  EXPECT_GT(0., result(1)); /* Ψ_ddot */
  EXPECT_GT(0., result(2)); /* β_dot */
}

}  // namespace
}  // namespace automotive
}  // namespace drake
