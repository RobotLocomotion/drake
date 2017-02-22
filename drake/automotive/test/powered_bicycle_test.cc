#include "drake/automotive/powered_bicycle.h"

#include <memory>

#include "gtest/gtest.h"

using std::make_unique;

namespace drake {
namespace automotive {
namespace {

class PoweredBicycleTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new PoweredBicycle<double>());

    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
    steering_input_.reset(new systems::BasicVector<double>(
        steering_input_dimension_));
    throttle_input_.reset(new systems::BasicVector<double>(
        throttle_input_dimension_));
  }

  systems::VectorBase<double>* continuous_state(
      const systems::System<double>* system) {
    systems::Context<double>* subcontext =
        dut_->GetMutableSubsystemContext(context_.get(), system);
    return subcontext->get_mutable_continuous_state_vector();
  }

  const systems::VectorBase<double>* state_derivatives(
      const systems::System<double>* system) {
    const systems::ContinuousState<double>* subderivatives =
        dut_->GetSubsystemDerivatives(*derivatives_, system);
    return &subderivatives->get_vector();
  }

  void SetInputs(const double& steering_input, const double& throttle_input) {
    DRAKE_DEMAND(steering_input_ != nullptr);
    DRAKE_DEMAND(throttle_input_ != nullptr);
    DRAKE_DEMAND(dut_ != nullptr);
    DRAKE_DEMAND(context_ != nullptr);

    (*steering_input_)[0] = steering_input;
    (*throttle_input_)[0] = throttle_input;

    const int kSteeringIndex = dut_->get_steering_input_port().get_index();
    const int kThrottleIndex = dut_->get_throttle_input_port().get_index();
    context_->FixInputPort(kSteeringIndex, std::move(steering_input_));
    context_->FixInputPort(kThrottleIndex, std::move(throttle_input_));
  }

  const int bike_state_dimension_{6};
  const int steering_input_dimension_{1};
  const int throttle_input_dimension_{1};

  std::unique_ptr<PoweredBicycle<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
  std::unique_ptr<systems::BasicVector<double>> steering_input_;
  std::unique_ptr<systems::BasicVector<double>> throttle_input_;
};

TEST_F(PoweredBicycleTest, Topology) {
  ASSERT_EQ(2, dut_->get_num_input_ports());  // Steering angle, throttle input.

  const int kSteeringIndex = dut_->get_steering_input_port().get_index();
  const auto& steering_input_descriptor = dut_->get_input_port(kSteeringIndex);
  EXPECT_EQ(systems::kVectorValued, steering_input_descriptor.get_data_type());
  EXPECT_EQ(steering_input_dimension_, steering_input_descriptor.size());

  const int kThrottleIndex = dut_->get_throttle_input_port().get_index();
  const auto& throttle_input_descriptor = dut_->get_input_port(kThrottleIndex);
  EXPECT_EQ(systems::kVectorValued, throttle_input_descriptor.get_data_type());
  EXPECT_EQ(throttle_input_dimension_, throttle_input_descriptor.size());

  ASSERT_EQ(1, dut_->get_num_output_ports()); /* bicycle state vector */

  const auto& state_descriptor = dut_->get_output_port(0);
  EXPECT_EQ(systems::kVectorValued, state_descriptor.get_data_type());
  EXPECT_EQ(bike_state_dimension_, state_descriptor.size());
}

TEST_F(PoweredBicycleTest, Output) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, output_);

  // Obtain pointers to the (continuous) state vector and output
  // vector for the bicycle.
  auto bike_state = continuous_state(dut_->get_bicycle_system());
  const systems::BasicVector<double>* output = output_->get_vector_data(0);

  dut_->CalcOutput(*context_, output_.get());

  // Define a set of assignments to the bicycle states.
  (*bike_state)[0] = 1.;
  (*bike_state)[1] = 2.;
  (*bike_state)[2] = 3.;
  (*bike_state)[3] = 4.;
  (*bike_state)[4] = 5.;
  (*bike_state)[5] = 6.;

  dut_->CalcOutput(*context_, output_.get());

  // Outputs agree with the states.
  EXPECT_EQ(1., (*output)[0]);
  EXPECT_EQ(2., (*output)[1]);
  EXPECT_EQ(3., (*output)[2]);
  EXPECT_EQ(4., (*output)[3]);
  EXPECT_EQ(5., (*output)[4]);
  EXPECT_EQ(6., (*output)[5]);
}

TEST_F(PoweredBicycleTest, TimeDerivatives) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, derivatives_);

  // Obtain pointers to the (continuous) state vector and state
  // derivative vector for each car.
  auto power_state = continuous_state(dut_->get_powertrain_system());
  auto bike_state = continuous_state(dut_->get_bicycle_system());

  auto power_derivatives = state_derivatives(dut_->get_powertrain_system());
  auto bike_derivatives = state_derivatives(dut_->get_bicycle_system());

  // Set the inputs to zero.
  SetInputs(0., 0.);

  // Set the states to arbitrary values (require a nonzero velocity).
  (*power_state)[0] = 6.0;
  (*bike_state)[0] = 27.0;
  (*bike_state)[1] = 7.0;
  (*bike_state)[2] = 6.0;
  (*bike_state)[3] = 10.0;
  (*bike_state)[4] = 31.0;
  (*bike_state)[5] = 42.0;

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  dut_->CalcOutput(*context_, output_.get());

  // Expected state derivatives.
  EXPECT_EQ(-15., (*power_derivatives)[0]);
  EXPECT_EQ(7., (*bike_derivatives)[0]);
  EXPECT_GE(0., (*bike_derivatives)[1]);
  EXPECT_GE(0., (*bike_derivatives)[2]);
  EXPECT_LE(0., (*bike_derivatives)[3]);
  EXPECT_GE(0., (*bike_derivatives)[4]);
  EXPECT_LE(0., (*bike_derivatives)[5]);
}

TEST_F(PoweredBicycleTest, PowerTrainTimeDerivativeAndOutput) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, derivatives_);

  // Obtain pointers to the (continuous) state vector and state
  // derivative vector for each car.
  auto power_state = continuous_state(dut_->get_powertrain_system());
  auto bike_state = continuous_state(dut_->get_bicycle_system());

  auto power_derivatives = state_derivatives(dut_->get_powertrain_system());
  auto bike_derivatives = state_derivatives(dut_->get_bicycle_system());

  // Set the throttle input to some positive value.
  SetInputs(0., 10.);

  // Set the power-train state to an arbitrary value. We also require
  // bicycle velocity to be nonzero.
  (*power_state)[0] = 20.;
  (*bike_state)[3] = 1.;

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  dut_->CalcOutput(*context_, output_.get());

  // Verify the state derivative.
  EXPECT_EQ(-40., (*power_derivatives)[0]);

  // Verify the v_dot for the bicycle model, as a surrugate for checking the
  // power-train output directly.
  EXPECT_LE(0., (*bike_derivatives)[3]);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
