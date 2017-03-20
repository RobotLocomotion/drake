#include "drake/automotive/bicycle_car.h"

#include <cmath>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace automotive {
namespace {

static constexpr int kStateDimension{BicycleCarStateIndices::kNumCoordinates};
static constexpr int kSteeringInputDimension{1};
static constexpr int kForceInputDimension{1};

class BicycleCarTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new BicycleCar<double>());

    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

  BicycleCarState<double>* continuous_state() {
    auto xc = context_->get_mutable_continuous_state_vector();
    BicycleCarState<double>* state = dynamic_cast<BicycleCarState<double>*>(xc);
    DRAKE_DEMAND(state != nullptr);
    return state;
  }

  const BicycleCarState<double>* derivatives() const {
    const auto derivatives = dynamic_cast<const BicycleCarState<double>*>(
        derivatives_->get_mutable_vector());
    DRAKE_DEMAND(derivatives != nullptr);
    return derivatives;
  }

  void SetInputs(const double steering_angle, const double force) {
    ASSERT_NE(nullptr, dut_);
    ASSERT_NE(nullptr, context_);

    std::unique_ptr<systems::BasicVector<double>> steering_input(
        new systems::BasicVector<double>(kSteeringInputDimension));
    std::unique_ptr<systems::BasicVector<double>> force_input(
        new systems::BasicVector<double>(kForceInputDimension));

    (*steering_input)[0] = steering_angle;
    (*force_input)[0] = force;

    const int kSteeringIndex = dut_->get_steering_input_port().get_index();
    const int kForceIndex = dut_->get_force_input_port().get_index();
    context_->FixInputPort(kSteeringIndex, std::move(steering_input));
    context_->FixInputPort(kForceIndex, std::move(force_input));
  }

  std::unique_ptr<BicycleCar<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

TEST_F(BicycleCarTest, Topology) {
  ASSERT_EQ(2, dut_->get_num_input_ports()); /* steering angle, force input */

  const auto& steering_input_descriptor = dut_->get_steering_input_port();
  EXPECT_EQ(systems::kVectorValued, steering_input_descriptor.get_data_type());
  EXPECT_EQ(kSteeringInputDimension, steering_input_descriptor.size());

  const auto& force_input_descriptor = dut_->get_force_input_port();
  EXPECT_EQ(systems::kVectorValued, force_input_descriptor.get_data_type());
  EXPECT_EQ(kForceInputDimension, force_input_descriptor.size());

  ASSERT_EQ(1, dut_->get_num_output_ports()); /* state vector */

  const auto& state_descriptor = dut_->get_output_port(0);
  EXPECT_EQ(systems::kVectorValued, state_descriptor.get_data_type());
  EXPECT_EQ(kStateDimension, state_descriptor.size());
}

TEST_F(BicycleCarTest, Output) {
  const double kTolerance = 1e-10;

  // Set the steering angle and the applied force to positive values.
  SetInputs(1., 10.);

  auto output = output_->get_vector_data(0);

  // Set all the states to one.
  continuous_state()->SetFromVector(Vector6<double>::Ones());

  dut_->CalcOutput(*context_, output_.get());
  const Vector6<double> result = output->CopyToVector();

  // Expect that the output matches the states, since there is no feedthrough.
  EXPECT_TRUE(CompareMatrices(result, Vector6<double>::Ones(), kTolerance,
                              MatrixCompareType::absolute));
}

// Tests the consistency of the derivatives when a trivial set of states is
// provided, with no steering and some positive input force.
TEST_F(BicycleCarTest, TrivialDerivatives) {
  const double kForceInput = 10.;
  const double kVelocityState = 1.;

  // Keep the steering angle zero and set the force to a positive value.
  SetInputs(0., kForceInput);

  // Set all the states to zero except velocity, which must be kept positive.
  continuous_state()->SetFromVector(Vector6<double>::Zero());
  continuous_state()->set_vel(kVelocityState);

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // We expect all derivatives to be zero except vel_dot and sx_dot, since
  // applying a positive force input and zero steering angle translates into
  // along-track motion.
  EXPECT_EQ(0., derivatives()->Psi());
  EXPECT_EQ(0., derivatives()->Psi_dot());
  EXPECT_EQ(0., derivatives()->beta());
  EXPECT_LT(0., derivatives()->vel());
  EXPECT_EQ(kVelocityState, derivatives()->sx());
  EXPECT_EQ(0., derivatives()->sy());
}

// Tests that one equation has terms that cancel for some parameter-independent
// settings, and that the remaining equations are still consistent, including
// the kinematics equations.
TEST_F(BicycleCarTest, DerivativesPositiveBetaPositiveDelta) {
  const double kSteeringInput = 1.;  // An angle in the first quadrant.
  const double kForceInput = 10.;
  const double kVelocityState = 1.;

  // Set the steering angle and force to positive values.
  SetInputs(kSteeringInput, kForceInput);

  // Set β to δ / 2 and the velocity to a positive value.
  continuous_state()->SetFromVector(Vector6<double>::Zero());
  continuous_state()->set_beta(kSteeringInput / 2.);
  continuous_state()->set_vel(kVelocityState);

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // We expect the first and third terms in β_dot to cancel, and Ψ_ddot, vel_dot
  // to be strictly positive. We expect sx_dot, and sy_dot to be
  // strictly-positive as the steering angle is in the first quadrant.
  EXPECT_LT(0., derivatives()->Psi_dot());
  EXPECT_EQ(0., derivatives()->beta());
  EXPECT_LT(0., derivatives()->vel());
  EXPECT_LT(0., derivatives()->sx());
  EXPECT_LT(0., derivatives()->sy());
}

// Tests the consistency of the derivatives and kinematic relationships upon
// feeding in a negative slip angle, keeping the other parameters the same as
// the previous case.
TEST_F(BicycleCarTest, DerivativesNegativeBetaPositiveDelta) {
  const double kSteeringInput = 1.;  // An angle in the fourth quadrant.
  const double kForceInput = 10.;
  const double kVelocityState = 1.;

  // Set the steering angle and force to positive values.
  SetInputs(kSteeringInput, kForceInput);

  // Set β to -δ and the velocity to a positive value.
  continuous_state()->SetFromVector(Vector6<double>::Zero());
  continuous_state()->set_beta(-kSteeringInput);
  continuous_state()->set_vel(kVelocityState);

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // We expect β_dot and vel_dot to return strictly-positive values. sx_dot and
  // sy_dot, respectively, return positive and negative values, as the steering
  // angle is in the fourth quadrant.  Note that Ψ_ddot is indeterminate.
  EXPECT_LT(0., derivatives()->beta());
  EXPECT_LT(0., derivatives()->vel());
  EXPECT_LT(0., derivatives()->sx());
  EXPECT_GT(0., derivatives()->sy());
}

// Tests the consistency of the derivatives and kinematic relationships upon
// assigning a positive angular yaw rate.
TEST_F(BicycleCarTest, DerivativesPositivePsiDot) {
  const double kYawRateState = 2.;
  const double kVelocityState = 1e3;  // Unrealistic velocity that reasonably
                                      // enforces the condition that
                                      // abs(Cr * lr - Cf * lf) < 1e6 * mass.

  // Keep the steering angle and force zero.
  SetInputs(0., 0.);

  // Set Ψ_dot and the velocity to positive values.
  continuous_state()->SetFromVector(Vector6<double>::Zero());
  continuous_state()->set_Psi_dot(kYawRateState);
  continuous_state()->set_vel(kVelocityState);

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  // We expect both β_dot and Ψ_ddot to be strictly negative.
  EXPECT_GT(0., derivatives()->Psi_dot());
  EXPECT_GT(0., derivatives()->beta());
}

}  // namespace
}  // namespace automotive
}  // namespace drake
