#include "drake/automotive/powered_bicycle.h"

#include <memory>

#include "gtest/gtest.h"

using std::make_unique;

namespace drake {
namespace automotive {
namespace {

// Specify the dimension of the state vector and of each input port.
static constexpr int kBikeStateDimension{BicycleStateIndices::kNumCoordinates};
static constexpr int kSteeringInputDimension{1};
static constexpr int kThrottleInputDimension{1};

class PoweredBicycleTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dut_.reset(new PoweredBicycle<double>());

    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
    steering_input_.reset(
        new systems::BasicVector<double>(kSteeringInputDimension));
    throttle_input_.reset(
        new systems::BasicVector<double>(kThrottleInputDimension));

    steering_index_ = dut_->get_steering_input_port().get_index();
    throttle_index_ = dut_->get_throttle_input_port().get_index();
    powertrain_index_ = dut_->get_powertrain_output_port().get_index();
    bike_state_index_ = dut_->get_state_output_port().get_index();
  }

  systems::VectorBase<double>* powertrain_continuous_state() {
    return get_continuous_state(dut_->get_powertrain_system());
  }

  BicycleState<double>* bicycle_continuous_state() {
    auto context_state = get_continuous_state(dut_->get_bicycle_system());
    BicycleState<double>* state =
        dynamic_cast<BicycleState<double>*>(context_state);
    DRAKE_DEMAND(state != nullptr);
    return state;
  }

  const systems::VectorBase<double>* powertrain_state_derivatives() const {
    return get_state_derivatives(dut_->get_powertrain_system());
  }

  const BicycleState<double>* bicycle_state_derivatives() const {
    const auto context_derivatives =
        get_state_derivatives(dut_->get_bicycle_system());
    const BicycleState<double>* derivatives =
        dynamic_cast<const BicycleState<double>*>(context_derivatives);
    DRAKE_DEMAND(derivatives != nullptr);
    return derivatives;
  }

  void SetInputs(const double& steering_input, const double& throttle_input) {
    ASSERT_NE(nullptr, steering_input_);
    ASSERT_NE(nullptr, throttle_input_);
    ASSERT_NE(nullptr, dut_);
    ASSERT_NE(nullptr, context_);

    (*steering_input_)[0] = steering_input;
    (*throttle_input_)[0] = throttle_input;

    context_->FixInputPort(steering_index_, std::move(steering_input_));
    context_->FixInputPort(throttle_index_, std::move(throttle_input_));
  }

  int steering_index_{};
  int throttle_index_{};
  int powertrain_index_{};
  int bike_state_index_{};

  std::unique_ptr<PoweredBicycle<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
  std::unique_ptr<systems::BasicVector<double>> steering_input_;
  std::unique_ptr<systems::BasicVector<double>> throttle_input_;

 private:
  systems::VectorBase<double>* get_continuous_state(
      const systems::System<double>* system) {
    systems::Context<double>* subcontext =
        dut_->GetMutableSubsystemContext(context_.get(), system);
    return subcontext->get_mutable_continuous_state_vector();
  }

  const systems::VectorBase<double>* get_state_derivatives(
      const systems::System<double>* system) const {
    const systems::ContinuousState<double>* subderivatives =
        dut_->GetSubsystemDerivatives(*derivatives_, system);
    return &subderivatives->get_vector();
  }
};

TEST_F(PoweredBicycleTest, Topology) {
  ASSERT_EQ(2, dut_->get_num_input_ports());  // Steering angle, throttle input.

  const auto& steering_input_descriptor = dut_->get_input_port(steering_index_);
  EXPECT_EQ(systems::kVectorValued, steering_input_descriptor.get_data_type());
  EXPECT_EQ(kSteeringInputDimension, steering_input_descriptor.size());

  const auto& throttle_input_descriptor = dut_->get_input_port(throttle_index_);
  EXPECT_EQ(systems::kVectorValued, throttle_input_descriptor.get_data_type());
  EXPECT_EQ(kThrottleInputDimension, throttle_input_descriptor.size());

  ASSERT_EQ(2, dut_->get_num_output_ports());  // powertrain state, bicycle
                                               // state vector

  const auto& powertrain_descriptor = dut_->get_output_port(powertrain_index_);
  EXPECT_EQ(systems::kVectorValued, powertrain_descriptor.get_data_type());
  EXPECT_EQ(1, powertrain_descriptor.size());

  const auto& state_descriptor = dut_->get_output_port(bike_state_index_);
  EXPECT_EQ(systems::kVectorValued, state_descriptor.get_data_type());
  EXPECT_EQ(kBikeStateDimension, state_descriptor.size());
}

TEST_F(PoweredBicycleTest, Output) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, output_);

  // Obtain pointers to the (continuous) state vector and output
  // vector for the bicycle.
  auto bike_state = bicycle_continuous_state();

  const systems::BasicVector<double>* output =
      output_->get_vector_data(bike_state_index_);
  const BicycleState<double>* bike_output =
      dynamic_cast<const BicycleState<double>*>(output);
  ASSERT_NE(nullptr, bike_output);

  // Set the inputs to zero.
  SetInputs(0., 0.);

  dut_->CalcOutput(*context_, output_.get());

  // Define a set of assignments to the bicycle states.
  bike_state->set_Psi(1.);
  bike_state->set_Psi_dot(2.);
  bike_state->set_beta(3.);
  bike_state->set_v(4.);
  bike_state->set_sx(5.);
  bike_state->set_sy(6.);

  dut_->CalcOutput(*context_, output_.get());

  // Outputs agree with the states.
  EXPECT_EQ(1., bike_output->Psi());
  EXPECT_EQ(2., bike_output->Psi_dot());
  EXPECT_EQ(3., bike_output->beta());
  EXPECT_EQ(4., bike_output->v());
  EXPECT_EQ(5., bike_output->sx());
  EXPECT_EQ(6., bike_output->sy());
}

TEST_F(PoweredBicycleTest, TimeDerivatives) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, derivatives_);

  // Obtain pointers to the (continuous) state vector and state
  // derivative vector for each car.
  auto power_state = powertrain_continuous_state();
  auto bike_state = bicycle_continuous_state();

  const auto power_derivatives = powertrain_state_derivatives();
  const auto bike_derivatives = bicycle_state_derivatives();

  // Set the inputs to zero.
  SetInputs(0., 0.);

  // Set the states to arbitrary values (require a nonzero velocity).
  (*power_state)[0] = 6.0;
  bike_state->set_Psi(27.);
  bike_state->set_Psi_dot(7.);
  bike_state->set_beta(6.);
  bike_state->set_v(10.);
  bike_state->set_sx(31.);
  bike_state->set_sy(42.);

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  dut_->CalcOutput(*context_, output_.get());

  // Expected state derivatives.
  EXPECT_EQ(-30., (*power_derivatives)[0]);
  EXPECT_EQ(7., bike_derivatives->Psi());
  EXPECT_GE(0., bike_derivatives->Psi_dot());
  EXPECT_GE(0., bike_derivatives->beta());
  EXPECT_LE(0., bike_derivatives->v());
  EXPECT_GE(0., bike_derivatives->sx());
  EXPECT_LE(0., bike_derivatives->sy());
}

TEST_F(PoweredBicycleTest, PowerTrainTimeDerivativeAndOutput) {
  ASSERT_NE(nullptr, context_);
  ASSERT_NE(nullptr, derivatives_);

  const double tc = dut_->get_powertrain_system()->get_time_constant();

  // Obtain pointers to the (continuous) state vector and state
  // derivative vector for each car.
  auto power_state = powertrain_continuous_state();
  auto bike_state = bicycle_continuous_state();

  const auto power_derivatives = powertrain_state_derivatives();
  const auto bike_derivatives = bicycle_state_derivatives();

  const systems::BasicVector<double>* power_output =
      output_->get_vector_data(powertrain_index_);

  // Set the throttle input to some positive value.
  SetInputs(0., 10.);

  // Set the powertrain state to an arbitrary value. We also require
  // bicycle velocity to be nonzero.
  (*power_state)[0] = 20.;
  bike_state->set_v(1.);

  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  dut_->CalcOutput(*context_, output_.get());

  // Verify the state derivative.
  EXPECT_EQ(-90., (*power_derivatives)[0]);

  // Verify the v_dot for the bicycle model, as a surrugate for checking the
  // powertrain output directly.
  EXPECT_LE(0., bike_derivatives->v());

  // Verify that the powertrain state matches its output.
  EXPECT_EQ((*power_state)[0], tc * (*power_output)[0]);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
